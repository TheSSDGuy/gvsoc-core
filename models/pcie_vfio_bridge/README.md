# GVSoC PCIe VFIO Bridge Module

## Overview

This module provides a **VFIO-user based PCIe endpoint bridge** between **GVSoC** and **QEMU**.  
The goal is to allow a simulated SoC inside GVSoC to expose a PCIe device that can be attached to a virtual machine running in QEMU.

In this setup:

```
+-------------------+          vfio-user socket           +------------------+
|       GVSoC       |  <------------------------------->  |       QEMU       |
|                   |                                     |                  |
|  PCIe endpoint    |                                     |  PCIe root port  |
|  (vfio-user)      |                                     |  guest VM        |
+-------------------+                                     +------------------+
```

The communication happens through **libvfio-user**, which exposes a PCIe device over a UNIX socket.

The QEMU VM sees the device as a **standard PCIe device**, allowing Linux drivers and applications to interact with it.

---

# Architecture

The bridge is implemented in the file:

```
pcie_vfio_mem_bridge.cpp
```

This component behaves as a **PCIe endpoint exposed through VFIO-user**.

Main responsibilities:

* Create and manage the **vfio-user server**
* Expose **PCIe BAR regions**
* Handle **MMIO transactions**
* Forward requests between **QEMU and the simulated device**
* Synchronize execution between GVSoC and QEMU

---

# Execution Model

The bridge uses a **blocking model** where:

1. GVSoC starts the VFIO-user server
2. QEMU connects through a UNIX socket
3. GVSoC waits for requests coming from QEMU

This means that:

**GVSoC will block waiting for QEMU to connect.**

This is expected behaviour.

Typical flow:

```
Start GVSoC
      ↓
VFIO-user server created
      ↓
GVSoC waits for connection
      ↓
Start QEMU
      ↓
QEMU connects to socket
      ↓
Simulation proceeds
```

---

# Memory Model (Current Status)

⚠ **Important**

The **memory integration inside GVSoC is not fully implemented yet.**

Currently:

* BAR regions exist
* MMIO transactions can be received
* But **memory accesses are not fully connected to the GVSoC memory hierarchy yet**

This means that the current implementation is mainly intended for:

* **infrastructure validation**
* **PCIe protocol integration**
* **DMA experiments from the host side**

Future work will connect the bridge to the **MAGIA memory system (L2 / TCDM)**.

---

# Dependencies

This module requires the **libvfio-user** library.

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

Compiler flags used in GVSoC:

```
-I/usr/local/include/vfio-user
-L/usr/local/lib/x86_64-linux-gnu
-lvfio-user
```

---

# GVSoC Integration

## 1 — vp_models.cmake modification

A modification was introduced in **vp_models.cmake**.

The goal is to allow **child CMakeLists to expose the module name** so that GVSoC can correctly build dynamic modules.

This allows modules to be defined locally without requiring manual duplication of names in parent build files.

---

## 2 — gvsoc/core/models/CMakeLists.txt

The module must be added to the build system.

Add the following line:

```cmake
add_subdirectory(pcie_vfio_bridge)
```

This registers the module inside the GVSoC build system.

---

# Typical Execution Flow

1. Start GVSoC (VFIO server)
2. Start QEMU with vfio-user device
3. QEMU connects to the socket
4. MMIO and DMA traffic flows between VM and GVSoC

---

## Host-side Test Environment

The PCIe VFIO bridge implemented in this module is designed to be used together with the host-side test environment available in the following repository:

https://github.com/TheSSDGuy/gvsoc-vfio-test

This repository contains the software components required to interact with the simulated PCIe device exposed by GVSoC, including:

- a Linux **kernel module** used to expose the VFIO device to user space
- a **DMA test application** used to validate data transfers between host memory and the simulated device
- an **ELF loader** used to load binaries into the accelerator memory through PCIe BAR accesses

### Typical Workflow

1. Start **GVSoC** with the `pcie_vfio_bridge` module enabled.
2. Start **QEMU** with the `vfio-user-pci` device connected to the GVSoC socket.
3. Boot the **Debian VM** inside QEMU.
4. Clone and build the host test repository.
5. Load the kernel module and run the DMA or ELF loader tests.

This setup enables development and validation of the **complete software stack** before running on real hardware, including:

- Linux driver development
- PCIe BAR access validation
- DMA transfers between host and accelerator
- program loading into the simulated device
