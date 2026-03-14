/*
 * Copyright (C) 2025 Fondazione Chips-IT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors: Lorenzo Zuolo, Chips-IT (lorenzo.zuolo@chips.it)
 *
 * Phase 1 notes:
 * - BAR0 is the DMA control BAR and also hosts a tiny MSI-X table/PBA area.
 * - BAR2 is the emulated card-local memory window.
 * - DMA execution is never performed inside the BAR0 MMIO callback.
 * - The BAR0 callback only latches a pending DMA request.
 * - The VFIO server thread runs in non-blocking attach mode, uses poll(),
 *   services vfio-user traffic, and executes pending DMA requests outside the
 *   MMIO callback path.
 * - Completion is reported through BAR0 status bits and optionally via MSI-X
 *   vector 0 when DMA_CTRL_IRQ_EN is set.
 *
 * Important note:
 * - This implementation does NOT use any vfu_sgl_* API.
 * - DMA accesses rely exclusively on direct DMA mappings received through
 *   vfu_setup_device_dma(), in the same spirit used by gem5: each DMA map is
 *   recorded as an {iova, size, vaddr, prot} tuple and later translated into a
 *   host pointer directly.
 */

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <vp/itf/wire.hpp>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <condition_variable>

#ifndef _Static_assert
#define _Static_assert static_assert
#endif

extern "C" {
#include <vfio-user/libvfio-user.h>
}

#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

#ifndef PCI_CAP_ID_MSIX
#define PCI_CAP_ID_MSIX 0x11
#endif

class PCIeVfioMemBridge : public vp::Component
{
public:
    PCIeVfioMemBridge(vp::ComponentConf &conf);

private:
    /* GVSoC interfaces */
    vp::IoSlave input_itf;
    vp::WireSlave<bool> done_irq_itf;
    vp::WireMaster<bool> fetch_enable_itf;
    vp::WireMaster<uint64_t> entry_addr_itf;

    /* VFIO */
    vfu_ctx_t *vfu_ctx = nullptr;

    uint64_t bar0_size = 0;
    uint64_t bar2_size = 0;

    std::vector<uint8_t> bar0;
    std::vector<uint8_t> bar2;

    std::string socket_path;
    std::mutex mutex;
    std::condition_variable conn_cv;
    bool connected = false;
    bool msix_vector0_masked = false;

    enum : uint64_t {
        BAR0_DMA_SRC_ADDR_LO = 0x00,
        BAR0_DMA_SRC_ADDR_HI = 0x04,
        BAR0_DMA_DST_ADDR_LO = 0x08,
        BAR0_DMA_DST_ADDR_HI = 0x0C,
        BAR0_DMA_LEN         = 0x10,
        BAR0_DMA_CTRL        = 0x14,
        BAR0_DMA_STATUS      = 0x18,
        BAR0_DMA_ERROR       = 0x1C,
        BAR0_DMA_MAGIC       = 0x20,
        BAR0_DMA_DIRECTION   = 0x24,

        /* Tiny MSI-X table/PBA placement inside BAR0. */
        BAR0_MSIX_TABLE_OFF  = 0x40,
        BAR0_MSIX_PBA_OFF    = 0x80,
    };

    enum : uint32_t {
        DMA_CTRL_START  = 1u << 0,
        DMA_CTRL_ABORT  = 1u << 1,
        DMA_CTRL_IRQ_EN = 1u << 2,
    };

    enum : uint32_t {
        DMA_STATUS_BUSY  = 1u << 0,
        DMA_STATUS_DONE  = 1u << 1,
        DMA_STATUS_ERROR = 1u << 2,
    };

    enum : uint32_t {
        DMA_DIR_HOST_TO_CARD = 0,
        DMA_DIR_CARD_TO_HOST = 1,
    };

    enum : uint32_t {
        DMA_ERR_NONE          = 0,
        DMA_ERR_BAD_DIRECTION = 1,
        DMA_ERR_ZERO_LENGTH   = 2,
        DMA_ERR_CARD_RANGE    = 3,
        DMA_ERR_HOST_ADDR     = 4,
        DMA_ERR_HOST_IO       = 5,
        DMA_ERR_SGL_TOO_SMALL = 6,
        DMA_ERR_BUSY          = 7,
        DMA_ERR_HOST_PERM     = 8,
    };

    struct DmaRegs {
        uint64_t src_addr;
        uint64_t dst_addr;
        uint32_t len;
        uint32_t ctrl;
        uint32_t status;
        uint32_t error;
        uint32_t magic;
        uint32_t direction;
    };

    struct PendingDma {
        bool valid = false;
        uint64_t src_addr = 0;
        uint64_t dst_addr = 0;
        uint32_t len = 0;
        uint32_t ctrl = 0;
        uint32_t direction = DMA_DIR_HOST_TO_CARD;
    };

    struct __attribute__((packed)) MsixCap {
        uint8_t  cap_id;
        uint8_t  next;
        uint16_t msgctl;
        uint32_t table;
        uint32_t pba;
    };

    struct DmaMapping {
        uint64_t iova = 0;
        uint64_t size = 0;
        uint8_t *vaddr = nullptr;
        uint32_t prot = 0;
    };

    static constexpr uint32_t DMA_MAGIC_VALUE = 0x44504131; /* 'DPA1' */
    static constexpr uint32_t MSIX_TABLE_BIR = 0; /* BAR0 */
    static constexpr uint32_t MSIX_PBA_BIR   = 0; /* BAR0 */
    static constexpr uint32_t MSIX_TABLE_OFFSET = BAR0_MSIX_TABLE_OFF;
    static constexpr uint32_t MSIX_PBA_OFFSET   = BAR0_MSIX_PBA_OFF;
    static constexpr uint32_t NUM_MSIX_VECTORS  = 1;

    PendingDma pending_dma;
    std::vector<DmaMapping> dma_mappings;

    static vp::IoReqStatus req(vp::Block *__this, vp::IoReq *req);

    static ssize_t bar0_access(vfu_ctx_t *vfu_ctx,
                               char *buf,
                               size_t count,
                               loff_t offset,
                               bool is_write);

    static ssize_t bar2_access(vfu_ctx_t *vfu_ctx,
                               char *buf,
                               size_t count,
                               loff_t offset,
                               bool is_write);

    static int device_reset(vfu_ctx_t *vfu_ctx, vfu_reset_type_t type);
    static void dma_register_cb(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info);
    static void dma_unregister_cb(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info);
    static void irq_state_cb(vfu_ctx_t *vfu_ctx, uint32_t start, uint32_t count, bool mask);

    void vfio_server_thread();
    void process_pending_dma();
    void maybe_trigger_irq(uint32_t ctrl);

    uint32_t read32_nolock(uint64_t offset) const;
    void write32_nolock(uint64_t offset, uint32_t value);
    DmaRegs snapshot_dma_regs_nolock() const;
    void clear_dma_status_nolock();
    void schedule_dma_from_regs_nolock();
    int execute_dma_from_request(const PendingDma &req);
    int dma_copy_host_to_card(uint64_t host_addr, uint64_t card_addr, uint32_t len);
    int dma_copy_card_to_host(uint64_t card_addr, uint64_t host_addr, uint32_t len);
    uint8_t *get_host_ptr_nolock(uint64_t iova, uint32_t len, int prot);
};

PCIeVfioMemBridge::PCIeVfioMemBridge(vp::ComponentConf &conf)
: vp::Component(conf)
{
    this->input_itf.set_req_meth(&PCIeVfioMemBridge::req);

    this->new_slave_port("input", &this->input_itf);
    this->new_slave_port("done_irq", &this->done_irq_itf);
    this->new_master_port("fetch_en", &this->fetch_enable_itf);
    this->new_master_port("entry_addr", &this->entry_addr_itf);

    this->socket_path = this->get_js_config()->get_child_str("socket_path");
    this->bar0_size   = this->get_js_config()->get_child_int("bar0_size");
    this->bar2_size   = this->get_js_config()->get_child_int("bar2_size");

    if (bar0_size < 0x100) {
        throw std::runtime_error("bar0_size must be at least 0x100 for DMA control/MSI-X registers");
    }

    bar0.resize(bar0_size, 0);
    bar2.resize(bar2_size, 0);

    {
        std::lock_guard<std::mutex> lock(mutex);
        write32_nolock(BAR0_DMA_MAGIC, DMA_MAGIC_VALUE);
        write32_nolock(BAR0_DMA_DIRECTION, DMA_DIR_HOST_TO_CARD);
        write32_nolock(BAR0_DMA_STATUS, 0);
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_NONE);
        pending_dma.valid = false;
        connected = false;
        msix_vector0_masked = false;
        dma_mappings.clear();
    }

    unlink(socket_path.c_str());

    vfu_ctx = vfu_create_ctx(VFU_TRANS_SOCK,
                             socket_path.c_str(),
                             LIBVFIO_USER_FLAG_ATTACH_NB,
                             this,
                             VFU_DEV_TYPE_PCI);

    if (vfu_ctx == nullptr) {
        throw std::runtime_error("vfu_create_ctx failed");
    }

    if (vfu_pci_init(vfu_ctx, VFU_PCI_TYPE_CONVENTIONAL, PCI_HEADER_TYPE_NORMAL, 0) < 0) {
        throw std::runtime_error("vfu_pci_init failed");
    }

    vfu_pci_set_id(vfu_ctx, 0x1d1d, 0x0001, 0x0000, 0x0000);
    vfu_pci_set_class(vfu_ctx, 0x12, 0x00, 0x00);

    if (vfu_setup_device_reset_cb(vfu_ctx, &PCIeVfioMemBridge::device_reset) < 0) {
        throw std::runtime_error("vfu_setup_device_reset_cb failed");
    }

    if (vfu_setup_device_dma(vfu_ctx,
                             &PCIeVfioMemBridge::dma_register_cb,
                             &PCIeVfioMemBridge::dma_unregister_cb) < 0) {
        throw std::runtime_error("vfu_setup_device_dma failed");
    }

    if (vfu_setup_device_nr_irqs(vfu_ctx, VFU_DEV_MSIX_IRQ, NUM_MSIX_VECTORS) < 0) {
        throw std::runtime_error("vfu_setup_device_nr_irqs MSI-X failed");
    }

    if (vfu_setup_irq_state_callback(vfu_ctx, VFU_DEV_MSIX_IRQ, &PCIeVfioMemBridge::irq_state_cb) < 0) {
        throw std::runtime_error("vfu_setup_irq_state_callback MSI-X failed");
    }

    {
        MsixCap msix_cap {};
        msix_cap.cap_id = PCI_CAP_ID_MSIX;
        msix_cap.next   = 0;
        msix_cap.msgctl = static_cast<uint16_t>((NUM_MSIX_VECTORS - 1) & 0x7ff);
        msix_cap.table  = (MSIX_TABLE_OFFSET & ~0x7u) | MSIX_TABLE_BIR;
        msix_cap.pba    = (MSIX_PBA_OFFSET   & ~0x7u) | MSIX_PBA_BIR;

        if (vfu_pci_add_capability(vfu_ctx, 0, 0, &msix_cap) < 0) {
            throw std::runtime_error("vfu_pci_add_capability MSI-X failed");
        }
    }

    if (vfu_setup_region(vfu_ctx,
                         VFU_PCI_DEV_BAR0_REGION_IDX,
                         bar0_size,
                         bar0_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_MEM,
                         NULL,
                         0,
                         -1,
                         0) < 0) {
        throw std::runtime_error("vfu_setup_region BAR0 failed");
    }

    if (vfu_setup_region(vfu_ctx,
                         VFU_PCI_DEV_BAR2_REGION_IDX,
                         bar2_size,
                         bar2_access,
                         VFU_REGION_FLAG_RW | VFU_REGION_FLAG_MEM,
                         NULL,
                         0,
                         -1,
                         0) < 0) {
        throw std::runtime_error("vfu_setup_region BAR2 failed");
    }

    if (vfu_realize_ctx(vfu_ctx) < 0) {
        throw std::runtime_error("vfu_realize_ctx failed");
    }

    std::cout << "Waiting for QEMU connection on " << socket_path << std::endl;

    std::thread(&PCIeVfioMemBridge::vfio_server_thread, this).detach();

    {
        std::unique_lock<std::mutex> lock(mutex);
        conn_cv.wait(lock, [this]() { return connected; });
    }

    std::cout << "QEMU connected!" << std::endl;
}

uint32_t PCIeVfioMemBridge::read32_nolock(uint64_t offset) const
{
    uint32_t value = 0;
    std::memcpy(&value, &bar0.at(offset), sizeof(value));
    return value;
}

void PCIeVfioMemBridge::write32_nolock(uint64_t offset, uint32_t value)
{
    std::memcpy(&bar0.at(offset), &value, sizeof(value));
}

PCIeVfioMemBridge::DmaRegs PCIeVfioMemBridge::snapshot_dma_regs_nolock() const
{
    DmaRegs regs;
    regs.src_addr = (static_cast<uint64_t>(read32_nolock(BAR0_DMA_SRC_ADDR_HI)) << 32) |
                    static_cast<uint64_t>(read32_nolock(BAR0_DMA_SRC_ADDR_LO));
    regs.dst_addr = (static_cast<uint64_t>(read32_nolock(BAR0_DMA_DST_ADDR_HI)) << 32) |
                    static_cast<uint64_t>(read32_nolock(BAR0_DMA_DST_ADDR_LO));
    regs.len = read32_nolock(BAR0_DMA_LEN);
    regs.ctrl = read32_nolock(BAR0_DMA_CTRL);
    regs.status = read32_nolock(BAR0_DMA_STATUS);
    regs.error = read32_nolock(BAR0_DMA_ERROR);
    regs.magic = read32_nolock(BAR0_DMA_MAGIC);
    regs.direction = read32_nolock(BAR0_DMA_DIRECTION);
    return regs;
}

void PCIeVfioMemBridge::clear_dma_status_nolock()
{
    write32_nolock(BAR0_DMA_STATUS, 0);
    write32_nolock(BAR0_DMA_ERROR, DMA_ERR_NONE);
}

void PCIeVfioMemBridge::schedule_dma_from_regs_nolock()
{
    DmaRegs regs = snapshot_dma_regs_nolock();

    if (pending_dma.valid) {
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_BUSY);
        write32_nolock(BAR0_DMA_STATUS, DMA_STATUS_DONE | DMA_STATUS_ERROR);
        write32_nolock(BAR0_DMA_CTRL, regs.ctrl & ~DMA_CTRL_START);
        return;
    }

    pending_dma.valid = true;
    pending_dma.src_addr = regs.src_addr;
    pending_dma.dst_addr = regs.dst_addr;
    pending_dma.len = regs.len;
    pending_dma.ctrl = regs.ctrl;
    pending_dma.direction = regs.direction;

    clear_dma_status_nolock();
    write32_nolock(BAR0_DMA_STATUS, DMA_STATUS_BUSY);
    write32_nolock(BAR0_DMA_CTRL, regs.ctrl & ~DMA_CTRL_START);

    std::cout << "DMA scheduled: dir=" << pending_dma.direction
              << " src=0x" << std::hex << pending_dma.src_addr
              << " dst=0x" << pending_dma.dst_addr
              << " len=0x" << pending_dma.len
              << std::dec << std::endl;
}

int PCIeVfioMemBridge::device_reset(vfu_ctx_t *vfu_ctx, vfu_reset_type_t type)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    std::cout << "VFIO device reset requested (type=" << type << ")" << std::endl;

    std::memset(bridge->bar0.data(), 0, bridge->bar0.size());
    bridge->write32_nolock(BAR0_DMA_MAGIC, DMA_MAGIC_VALUE);
    bridge->write32_nolock(BAR0_DMA_DIRECTION, DMA_DIR_HOST_TO_CARD);
    bridge->write32_nolock(BAR0_DMA_STATUS, 0);
    bridge->write32_nolock(BAR0_DMA_ERROR, DMA_ERR_NONE);
    bridge->pending_dma.valid = false;
    bridge->msix_vector0_masked = false;
    bridge->dma_mappings.clear();

    return 0;
}

void PCIeVfioMemBridge::dma_register_cb(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    DmaMapping mapping;
    mapping.iova = reinterpret_cast<uintptr_t>(info->iova.iov_base);
    mapping.size = info->iova.iov_len;
    mapping.vaddr = reinterpret_cast<uint8_t *>(info->vaddr);
    mapping.prot = info->prot;

    std::cout << "DMA map: iova=0x" << std::hex << mapping.iova
              << " len=0x" << mapping.size
              << " vaddr=" << static_cast<void *>(mapping.vaddr)
              << " prot=0x" << mapping.prot
              << std::dec << std::endl;

    bridge->dma_mappings.push_back(mapping);
}

void PCIeVfioMemBridge::dma_unregister_cb(vfu_ctx_t *vfu_ctx, vfu_dma_info_t *info)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    const uint64_t iova = reinterpret_cast<uintptr_t>(info->iova.iov_base);
    const uint64_t size = info->iova.iov_len;

    std::cout << "DMA unmap: iova=0x" << std::hex << iova
              << " len=0x" << size
              << std::dec << std::endl;

    bridge->dma_mappings.erase(
        std::remove_if(bridge->dma_mappings.begin(),
                       bridge->dma_mappings.end(),
                       [iova, size](const DmaMapping &m) {
                           return m.iova == iova && m.size == size;
                       }),
        bridge->dma_mappings.end());
}

void PCIeVfioMemBridge::irq_state_cb(vfu_ctx_t *vfu_ctx, uint32_t start, uint32_t count, bool mask)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    if (start == 0 && count > 0) {
        bridge->msix_vector0_masked = mask;
        std::cout << "MSI-X vector0 " << (mask ? "masked" : "unmasked") << std::endl;
    }
}

vp::IoReqStatus PCIeVfioMemBridge::req(vp::Block *__this, vp::IoReq *req)
{
    PCIeVfioMemBridge *_this = static_cast<PCIeVfioMemBridge *>(__this);
    std::lock_guard<std::mutex> lock(_this->mutex);

    uint64_t addr = req->get_addr();
    uint8_t *data = req->get_data();
    size_t size = req->get_size();

    if (addr + size > _this->bar2.size()) {
        return vp::IO_REQ_INVALID;
    }

    if (req->get_is_write()) {
        std::memcpy(&_this->bar2[addr], data, size);
    } else {
        std::memcpy(data, &_this->bar2[addr], size);
    }

    return vp::IO_REQ_OK;
}

uint8_t *PCIeVfioMemBridge::get_host_ptr_nolock(uint64_t iova, uint32_t len, int prot)
{
    const uint64_t end = iova + static_cast<uint64_t>(len);

    for (const auto &mapping : dma_mappings) {
        const uint64_t map_start = mapping.iova;
        const uint64_t map_end = mapping.iova + mapping.size;

        if (iova >= map_start && end <= map_end) {
            if (mapping.vaddr == nullptr) {
                return nullptr;
            }
            if ((mapping.prot & prot) != static_cast<uint32_t>(prot)) {
                errno = EACCES;
                return nullptr;
            }
            return mapping.vaddr + (iova - map_start);
        }
    }

    errno = ENOENT;
    return nullptr;
}

int PCIeVfioMemBridge::dma_copy_host_to_card(uint64_t host_addr,
                                             uint64_t card_addr,
                                             uint32_t len)
{
    std::cout << "DMA H2C start: host=0x" << std::hex << host_addr
              << " card=0x" << card_addr
              << " len=0x" << len
              << std::dec << std::endl;

    if (len == 0) {
        std::lock_guard<std::mutex> lock(mutex);
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_ZERO_LENGTH);
        return -1;
    }

    std::lock_guard<std::mutex> lock(mutex);

    if (card_addr + len > bar2.size()) {
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_CARD_RANGE);
        return -1;
    }

    errno = 0;
    uint8_t *host_ptr = get_host_ptr_nolock(host_addr, len, PROT_READ);
    if (host_ptr == nullptr) {
        std::cout << "DMA H2C get_host_ptr failed errno=" << errno << std::endl;
        write32_nolock(BAR0_DMA_ERROR, errno == EACCES ? DMA_ERR_HOST_PERM : DMA_ERR_HOST_ADDR);
        return -1;
    }

    std::memcpy(&bar2[card_addr], host_ptr, len);

    std::cout << "DMA H2C done via direct mapping" << std::endl;
    return 0;
}

int PCIeVfioMemBridge::dma_copy_card_to_host(uint64_t card_addr,
                                             uint64_t host_addr,
                                             uint32_t len)
{
    std::cout << "DMA C2H start: card=0x" << std::hex << card_addr
              << " host=0x" << host_addr
              << " len=0x" << len
              << std::dec << std::endl;

    if (len == 0) {
        std::lock_guard<std::mutex> lock(mutex);
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_ZERO_LENGTH);
        return -1;
    }

    std::lock_guard<std::mutex> lock(mutex);

    if (card_addr + len > bar2.size()) {
        write32_nolock(BAR0_DMA_ERROR, DMA_ERR_CARD_RANGE);
        return -1;
    }

    errno = 0;
    uint8_t *host_ptr = get_host_ptr_nolock(host_addr, len, PROT_WRITE);
    if (host_ptr == nullptr) {
        std::cout << "DMA C2H get_host_ptr failed errno=" << errno << std::endl;
        write32_nolock(BAR0_DMA_ERROR, errno == EACCES ? DMA_ERR_HOST_PERM : DMA_ERR_HOST_ADDR);
        return -1;
    }

    std::memcpy(host_ptr, &bar2[card_addr], len);

    std::cout << "DMA C2H done via direct mapping" << std::endl;
    return 0;
}

int PCIeVfioMemBridge::execute_dma_from_request(const PendingDma &req)
{
    switch (req.direction) {
        case DMA_DIR_HOST_TO_CARD:
            return dma_copy_host_to_card(req.src_addr, req.dst_addr, req.len);

        case DMA_DIR_CARD_TO_HOST:
            return dma_copy_card_to_host(req.src_addr, req.dst_addr, req.len);

        default:
        {
            std::lock_guard<std::mutex> lock(mutex);
            write32_nolock(BAR0_DMA_ERROR, DMA_ERR_BAD_DIRECTION);
            return -1;
        }
    }
}

void PCIeVfioMemBridge::maybe_trigger_irq(uint32_t ctrl)
{
    if ((ctrl & DMA_CTRL_IRQ_EN) == 0) {
        return;
    }

    bool masked = false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        masked = msix_vector0_masked;
    }

    if (masked) {
        std::cout << "MSI-X vector0 masked, not triggering" << std::endl;
        return;
    }

    errno = 0;
    if (vfu_irq_trigger(vfu_ctx, 0) < 0) {
        std::cerr << "vfu_irq_trigger failed errno=" << errno << std::endl;
    } else {
        std::cout << "MSI-X vector0 triggered" << std::endl;
    }
}

void PCIeVfioMemBridge::process_pending_dma()
{
    PendingDma req;
    bool have_work = false;

    {
        std::lock_guard<std::mutex> lock(mutex);
        if (pending_dma.valid) {
            req = pending_dma;
            pending_dma.valid = false;
            have_work = true;
        }
    }

    if (!have_work) {
        return;
    }

    std::cout << "Processing pending DMA..." << std::endl;

    int ret = execute_dma_from_request(req);

    {
        std::lock_guard<std::mutex> lock(mutex);
        if (ret == 0) {
            write32_nolock(BAR0_DMA_STATUS, DMA_STATUS_DONE);
        } else {
            write32_nolock(BAR0_DMA_STATUS, DMA_STATUS_DONE | DMA_STATUS_ERROR);
        }
    }

    maybe_trigger_irq(req.ctrl);

    std::cout << "Pending DMA complete, ret=" << ret << std::endl;
}

ssize_t PCIeVfioMemBridge::bar0_access(vfu_ctx_t *vfu_ctx,
                                       char *buf,
                                       size_t count,
                                       loff_t offset,
                                       bool is_write)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    if (offset + count > bridge->bar0.size()) {
        errno = EINVAL;
        return -1;
    }

    if (is_write) {
        std::memcpy(&bridge->bar0[offset], buf, count);

        const bool touched_ctrl =
            (offset <= static_cast<loff_t>(BAR0_DMA_CTRL + 3)) &&
            (offset + count > static_cast<loff_t>(BAR0_DMA_CTRL));

        if (touched_ctrl) {
            const uint32_t ctrl = bridge->read32_nolock(BAR0_DMA_CTRL);

            if (ctrl & DMA_CTRL_ABORT) {
                bridge->pending_dma.valid = false;
                bridge->write32_nolock(BAR0_DMA_STATUS, DMA_STATUS_DONE | DMA_STATUS_ERROR);
                bridge->write32_nolock(BAR0_DMA_ERROR, DMA_ERR_NONE);
                bridge->write32_nolock(BAR0_DMA_CTRL, ctrl & ~(DMA_CTRL_START | DMA_CTRL_ABORT));
                std::cout << "DMA abort requested" << std::endl;
            } else if ((ctrl & DMA_CTRL_START) && bridge->connected) {
                bridge->schedule_dma_from_regs_nolock();
            }
        }
    } else {
        std::memcpy(buf, &bridge->bar0[offset], count);
    }

    return static_cast<ssize_t>(count);
}

ssize_t PCIeVfioMemBridge::bar2_access(vfu_ctx_t *vfu_ctx,
                                       char *buf,
                                       size_t count,
                                       loff_t offset,
                                       bool is_write)
{
    PCIeVfioMemBridge *bridge = static_cast<PCIeVfioMemBridge *>(vfu_get_private(vfu_ctx));
    std::lock_guard<std::mutex> lock(bridge->mutex);

    if (offset + count > bridge->bar2.size()) {
        errno = EINVAL;
        return -1;
    }

    if (is_write) {
        std::memcpy(&bridge->bar2[offset], buf, count);
    } else {
        std::memcpy(buf, &bridge->bar2[offset], count);
    }

    return static_cast<ssize_t>(count);
}

void PCIeVfioMemBridge::vfio_server_thread()
{
    bool attached = false;

    while (true) {

        if (!attached) {
            errno = 0;
            int ret = vfu_attach_ctx(vfu_ctx);

            if (ret == 0) {
                attached = true;

                {
                    std::lock_guard<std::mutex> lock(mutex);
                    connected = true;
                }
                conn_cv.notify_all();

                std::cout << "vfio_server_thread: QEMU attached, poll_fd="
                          << vfu_get_poll_fd(vfu_ctx) << std::endl;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "vfu_attach_ctx failed, errno=" << errno << std::endl;
                break;
            }
        }

        process_pending_dma();

        int fd = vfu_get_poll_fd(vfu_ctx);
        if (fd < 0) {
            continue;
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        int timeout_ms = attached ? 1 : 50;
        int pret = poll(&pfd, 1, timeout_ms);

        if (pret < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "poll failed, errno=" << errno << std::endl;
            break;
        }

        if (pret == 0) {
            continue;
        }

        errno = 0;
        int ret = vfu_run_ctx(vfu_ctx);

        if (ret == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }

            if (errno == ENOTCONN) {
                std::cerr << "vfio client disconnected, waiting for re-attach..." << std::endl;

                attached = false;

                {
                    std::lock_guard<std::mutex> lock(mutex);
                    connected = false;
                    pending_dma.valid = false;
                    dma_mappings.clear();
                    write32_nolock(BAR0_DMA_STATUS, 0);
                    write32_nolock(BAR0_DMA_ERROR, DMA_ERR_NONE);
                }

                continue;
            }

            std::cerr << "VFIO error, errno=" << errno << std::endl;
            break;
        }
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new PCIeVfioMemBridge(config);
}
