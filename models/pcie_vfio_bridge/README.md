# GVSoC PCIe VFIO Bridge Module

## Overview

This module implements a **VFIO-user based PCIe endpoint bridge** between **GVSoC** and **QEMU**.
It allows a simulated accelerator or memory-facing device inside GVSoC to be exposed to a QEMU guest as a PCIe endpoint.

In this setup:

```
+-------------------+          vfio-user socket           +------------------+
|       GVSoC       |  <------------------------------->  |       QEMU       |
|                   |                                     |                  |
|  PCIe endpoint    |                                     |  PCIe root port  |
|  (vfio-user)      |                                     |  guest VM        |
+-------------------+                                     +------------------+
```

The transport layer is handled through **libvfio-user**, while device-side accesses are forwarded to the GVSoC model through native GVSoC interfaces.

---

## Architecture

The bridge implementation lives in:

```
pcie_vfio_mem_bridge.cpp
```

Main responsibilities:

- create and manage the **vfio-user** server
- expose **BAR0** as the control/MMIO region
- receive MMIO accesses from QEMU through libvfio-user
- forward DMA requests to the GVSoC memory system through the `mem` IO master port
- propagate `ENTRY_POINT` and `FETCH_ENABLE` updates to the accelerator-facing GVSoC interfaces
- report DMA completion through BAR status bits and optional MSI-X

### Ports exposed by the model

- `mem`: GVSoC IO master used for DMA accesses toward the device memory space
- `done_irq`: wire slave used by the accelerator to signal end-of-compute
- `fetch_en`: wire master driven from BAR0 `FETCH_ENABLE`
- `entry_addr`: wire master driven from BAR0 `ENTRY_POINT`

---

## Execution Model

The bridge now uses a **split execution model**:

- a dedicated **VFIO thread** runs the libvfio-user event loop
- the actual **GVSoC-side work** is executed back on the GVSoC thread through clock events

This is important because:

- BAR callbacks from libvfio-user do **not** directly access the GVSoC memory hierarchy
- MMIO writes only latch state and schedule a GVSoC event when needed
- DMA requests are executed from the GVSoC side through the `mem` port
- the VFIO thread remains responsible for transport-side polling, attach/re-attach handling and MSI-X triggering

### High-level flow

1. GVSoC starts the VFIO-user server
2. QEMU connects to the UNIX socket
3. QEMU performs BAR writes or DMA setup
4. the bridge latches the request
5. a GVSoC event executes the corresponding device-side action
6. completion is reflected back through BAR status and optional MSI-X

The component still waits for the QEMU side to attach during startup.

---

## DMA Model

The bridge supports two DMA directions:

- **host to card**
- **card to host**

DMA mappings are provided by libvfio-user through `vfu_setup_device_dma()`.
The bridge resolves guest IOVAs against those mappings and then issues a GVSoC IO request on the `mem` port.

Current behavior:

- DMA requests are programmed through BAR0 registers
- only one DMA request can be active at a time
- completion updates `DMA_STATUS` and `DMA_ERROR`
- MSI-X vector 0 can be triggered when `DMA_CTRL_IRQ_EN` is set
- abort cancels a pending request but does not forcibly cancel an already in-flight GVSoC request

---

## Accelerator Integration

Two BAR-controlled accelerator hooks are exposed:

- `ENTRY_POINT`: forwarded to `entry_addr`
- `FETCH_ENABLE`: forwarded to `fetch_en`

In addition, when the accelerator asserts `done_irq`:

- the bridge forces `fetch_en` low
- the bridge schedules a **full GVSoC reset** by asserting and releasing the top-level reset hierarchy

This behavior is intentional and reflects the current integration model implemented in the code.

---

## Dependencies

This module requires **libvfio-user**.

Repository:

https://github.com/nutanix/libvfio-user

Installation example:

```bash
git clone https://github.com/nutanix/libvfio-user
cd libvfio-user
meson build
ninja -C build
sudo ninja -C build install
```

Expected install paths:

```
/usr/local/include/vfio-user
/usr/local/lib/x86_64-linux-gnu
```

The module CMake file links directly against `vfio-user` and adds the required include path locally.

---

## GVSoC Integration

To register the module in the build, add it to:

```cmake
gvsoc/core/models/CMakeLists.txt
```

with:

```cmake
add_subdirectory(pcie_vfio_bridge)
```

No extra `vp_models.cmake` modification is required for this module anymore.

---

## Typical Execution Flow

1. Start GVSoC with the bridge enabled
2. Start QEMU with a `vfio-user-pci` endpoint connected to the bridge socket
3. Program BAR0 registers from the guest
4. Trigger DMA and accelerator-side execution as needed
5. Observe completion through BAR state, MSI-X and accelerator done signaling

---

## Host-side Test Environment

The bridge was designed to work together with the host-side test environment available here:

https://github.com/TheSSDGuy/gvsoc-vfio-test

This repository contains software components useful to validate the end-to-end flow, including:

- a Linux **kernel module** exposing the VFIO device to user space
- a **DMA test application**
- an **ELF loader** using PCIe BAR accesses

Typical workflow:

1. start **GVSoC** with the `pcie_vfio_bridge` module enabled
2. start **QEMU** with the `vfio-user-pci` device connected to the bridge socket
3. boot the guest VM
4. build and load the host-side software stack
5. run DMA and loader tests against the simulated endpoint
