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
* For most users with a bricked controller: using a OpenOCD with a JTAG adaper
* For advanced users, FPGA programmers: using Xilinx ISE 

See instructions on the `mechatronics-firmware wiki
<https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program>`_.


FPGA1394V3, ESPM and ESSJ
*************************

To avoid any confusion between all the different firmware files on SD cards, we
provide a single Zip file that includes the firmware for the FPGA1394V3, ESPM
and ESSJ. The loader find the correct firmware by name. This way, users can put
any SD card in any dVRK component. 

Remove all the SD cards from your system:

* For dVRK Classic V3 controllers, card slot is located on the back
* For dVRK Si controllers, card slot is on the front
* For the ESPM and ESSJ, the SD card is on the programmer itself

The simplest approach is to use the :ref:`dVRK SD card updater
<sd-card-updater>` to download all the dVRK firmware files to one or more SD
card(s). This script downloads the latest firmware files, unzip them and wait
for an SD card to be inserted in the PC.  When an SD card is inserted, it
detects and mounts the card.  The script then moves the existing files on the SD card to a
backup directory and installs the new files. It then unmounts the SD card and waits
for the next one to update. 

Alternatively, you can download all the latest firmware files in a Zip file,
uncompress and copy all the files to the SD card. Last version can be found in
https://github.com/jhu-cisst/mechatronics-embedded/releases/latest.


dSIB-Si and dSIB-Si-Z
*********************

Program over USB using Arduino IDE.  See https://github.com/jhu-dvrk/dSIB-Si-firmware.