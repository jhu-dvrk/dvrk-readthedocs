.. _firmware:

########
Firmware
########

Introduction
############

The recommended FPGA firmware version is **v10**. It is backward compatible
with all dVRK controllers.

.. note::

   This section is an overview, please follow the links provided below for in depth documentation.

There are a few boards with embedded firmware used for the dVRK, either Classic or Si:

* :ref:`FPGA1394 <fpga>`: FPGA based, designed at JHU for generic robot controllers. dVRK
  :ref:`Classic <controllers-classic>` and :ref:`Si <controllers-si>`, located
  in dVRK controllers. Both the board design and firmware are open-sourced.
  FPGA1394V1 and FPGA1394V2 boot from EPROM. FPGA1394V3 boots from a SD card.

* Power Indicator board: designed at WPI. dVRK Classic only. Boots from EPROM.

* :ref:`ESPM <espm>`: FPGA based, designed by Intuitive Surgical. dVRK Si only,
  located in PSM/ECM arm. Board design is not open-sourced, firmware is specific
  to dVRK and is not open-sourced. Boots from EPROM or custom dVRK bootloader
  with SD card.

* :ref:`ESSJ <essj>`: FPGA based, designed by Intuitive Surgical. dVRK Si with SUJ only,
  located in SUJ arms. Board design is not open-sourced, firmware is specific to
  dVRK and is not open-sourced.  Boots from EPROM or custom dVRK bootloader
  with SD card.

* :ref:`dSIB-Si and dSIB-Si-Z <dsib-si-setup>`: STM32 microprocessor based,
  designed at JHU for the :ref:`dVRK Si with SUJ <setup-si-suj>`.  Boards
  attached on the back of dVRK Si controllers to interface with the patient's
  cart internal wiring. Board design and firmware are open-sourced. Boots from
  EPROM. 

.. important::

   *FPGA* and *firmware* is used to reference the FPGA1394 board and firmware
   unless specified otherwise.

Upgrade
#######

FPGA1394V1 and V2
*****************

There are 3 main ways to upgrade the firmware for the dVRK Classic controllers FPGA1394V1 and FPGA1394V2:

* For most users: over FireWire
* For most users with a bricked controller: using OpenOCD with a JTAG adapter
* For advanced users, FPGA programmers: using Xilinx ISE 

See instructions on the `mechatronics-firmware wiki
<https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program>`_.

FPGA1394V3
**********

FPGA1394V3 is used for the dVRK-Si PSM and ECM controllers, and for the newer generation
of dVRK MTM controller with DQLA (dual QLA). It can also be used as a replacement
for FPGA1394V1 or FPGA1394V2 in an older generation dVRK controller, though this is not
normally done.
We provide a single Zip file that includes the firmware for all supported uses of FPGA1394V3.
The loader finds the correct firmware by name. This way, users can put
the SD card into any dVRK/dVRK-Si controller that has an SD slot. 

Remove all the SD cards from your system:

* For dVRK Classic V3 controllers, card slot is located on the back
* For dVRK Si controllers, card slot is on the front

The simplest approach is to use the :ref:`dVRK SD card updater
<sd-card-updater>` to download all the dVRK firmware files to one or more SD
card(s). This script downloads the latest firmware files, unzips them and waits
for an SD card to be inserted in the PC.  When an SD card is inserted, it
detects and mounts the card.  The script then moves the existing files on the SD card to a
backup directory and installs the new files. It then unmounts the SD card and waits
for the next one to update. 

Alternatively, you can download all the latest firmware files in a Zip file,
decompress and copy all the files to the SD card. The latest version can be found in
https://github.com/jhu-cisst/mechatronics-embedded/releases/latest.

If the controllers are connected to the PC via Ethernet, it is also possible to
update the SD card using `scp`.

dSIB-Si and dSIB-Si-Z
*********************

See:

* `dSIB-Si-firmware <https://github.com/jhu-dvrk/dSIB-Si-firmware>`_
* `dSIB-Z-Si-firmware <https://github.com/jhu-dvrk/dSIB-Z-Si-firmware>`_

