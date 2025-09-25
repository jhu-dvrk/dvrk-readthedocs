.. _firmware:

########
Firmware
########

Introduction
############

.. note::

   This section is an overview, please follow the links provided below for in depth documentation.

There are a few boards with embedded firmware used for the dVRK, either Classic or Si:

* :ref:`FPGA1394 <fpga>`: FPGA based, designed at JHU for generic robot controllers. dVRK
  :ref:`Classic <controllers-classic>` and :ref:`Si <controllers-Si>`, located
  in dVRK controllers. Both the board design and firmware are open-sourced.
  FPGA1394V1 and FPGA1394V2 boot from EPROM. FPGA1394V3 boots from a SD card.

* Power Indicator board: designed at WPI. dVRK Classic only. Boots from EPROM.

* :ref:`ESPM <espm>`: FPGA based, designed by Intuitive Surgical. dVRK Si only,
  located in PSM/ECM arm. Board design is not open-sourced, firmware is specific
  to dVRK and is not open-sourced. Boots from EPROM or custom dVRK bootloader
  with SD card.

* ESSJ: FPGA based, designed by Intuitive Surgical. dVRK Si with SUJ only,
  located in SUJ arm. Board design is not open-sourced, firmware is specific to
  dVRK and is not open-sourced.  Boots from EPROM or custom dVRK bootloader
  with SD card

* :ref:`dSIB-Si and dSIB-Si-Z <dsib-si-setup>`: STM32 microprocessor based,
  designed at JHU for the :ref:`dVRK Si with SUJ <setup-si-suj>`.  Boards
  attached on the back of dVRK Si controllers to interface with the patient's
  cart internal wiring. Board design and firmware are open-sourced. Boots from
  EPROM. 

.. important::

   *FPGA* and *firmware* is used to reference the FPGA1394 board and firmware
   unless specified otherwise.

.. hint::

    To avoid any confusion between all the different firmware files on SD
    cards, we provide a single zip file that includes the firmware for the
    FPGA1394V3, ESPM and ESSJ. The loader find the correct firmware by name.
    This way, users can put any SD card in any dVRK component. 

FPGA1394
########

Features
********

FPGA1394V1 and V2
=================

Each FPGA1394 (V1 or V2) has one Xilinx Spartan 6 FPGA.  Xilinx ISE is required
to build the firmware.

Each FPGA1394V1 (previously known as FPGA1394) has two FireWire ports to
communicate with the PC. FPGA1394V2 (previously known as FPGA1394-Eth), adds a
100Mb Ethernet port.

FPGA1394V1s and V2s can be mated to the QLA motor power boards. Due to the
limited number of IOs on the Spartan 6, it takes one FPGA board per QLA.

* Board design: https://github.com/jhu-cisst/FPGA1394
* Firmware and documentation for all FPGA1394 versions: https://github.com/jhu-cisst/mechatronics-firmware
* :ref:`dVRK Classic controller V1 or V2 <classic-internal-v12>`

FPGA1394V3
==========

Each FPGA1394V3 has one Xilinx Zynq 7000 SoC (FPGA + ARM processor).  Xilinx ISE
or Vivado can be used to build the firmware.

Each FPGA1394V3 has two FireWire ports as well as two 1Gb Ethernet ports.

FPGA1394V3s can be mated directly to any existing QLA motor power board.  It is
also possible to use a single FPGA1394V3 to control 2 QLA boards using the DQLA
adapter boards (dVRK Classic). Finally, FPGA1394V3 are the only boards
supporting the dRAC motor power boards (dVRK Si).

* Board design: https://github.com/jhu-cisst/FPGA1394V3
* Firmware and documentation for all FPGA1394 versions: https://github.com/jhu-cisst/mechatronics-firmware
* :ref:`dVRK Classic controller V3 <classic-internal-v3>`
* :ref:`dVRK Si controller <si-internal>`

Upgrade
*******

FPGA1394V1 and V2
=================

There are 3 main ways to upgrade the firmware for the dVRK Classic controllers FPGA1394V1 and FPGA1394V2:

* For most users: over FireWire
* For most users with a bricked controller: using a OpenOCD with a JTAG adaper
* For advanced users, FPGA programmers: using Xilinx ISE 

FireWire
--------

todo: pgm1394 and pgm1394multi.sh

More details can be found on https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program.

OpenOCD and JTAG
----------------

todo: document or link to new repo

Xilinx ISE
----------

The process is described in https://github.com/jhu-cisst/mechatronics-firmware.

FPGA1394V3
==========

Update the SD card in the controller (thin slot on the front).  The simplest
approach is to remove the card and then use the :ref:`dVRK SD card updater
<sd-card-updater>` to download all the dVRK firmwares to one or more SD card(s).

Last version can be found in https://github.com/jhu-cisst/mechatronics-embedded/releases/latest

ESPM and ESSJ
#############

Upgrade
*******

Update the SD card in the boot loader.  The simplest approach is to remove the
card and then use the :ref:`dVRK SD card updater <sd-card-updater>` to download
all the dVRK firmware to one or more SD card(s).

Last version can be found in https://github.com/jhu-cisst/mechatronics-embedded/releases/latest

dSIB-Si and dSIB-Si-Z
#####################

Features
********

* Board design: https://github.com/jhu-dvrk/dSIB-Si-pcba
* Firmware: https://github.com/jhu-dvrk/dSIB-Si-firmware

Upgrade
*******

Program over USB using Arduino IDE.