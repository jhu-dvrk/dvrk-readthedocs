
.. include:: /includes/logic-view-espm.rst

ESPM and ESSJ
#############

.. _espm:

ESPM
====

.. note::

   This step is required for all PSMs and ECM Si used with the dVRK
   controllers, whether they are mounted on the Si SUJ or not.

Introduction
************

The circuit board (ESPM) inside the arm manages all the low power
signals for the Si PSMs and ECMs: encoders, potentiometers, buttons,
instrument Id, LEDs...

By default, it communicates with the core controller using a
proprietary protocol over LVDS.  JHU, in collaboration with Intuitive
Surgical, developed a closed-source FPGA firmware for the ESPM that
relies on an open protocol. The firmware is publicly available in a
binary image only.

.. figure:: /images/Si/ESPM-single-arm.jpg
   :width: 400
   :align: center

   Single arm setup with 1 ESPM

Accessing the ESPM board
************************

You first need to make sure the arm is not powered.

Then remove the plastic cover on the robot arm. You will need an
imperial Allen wrench to remove the single bolt holding the cover.
You will then need to wiggle or pry-out the cover since it's also held
by a few clips.

.. figure:: /images/Si/ESPM-cover.jpg
   :width: 400
   :align: center

   ESPM cover

.. note::

   If the arm is folded, and you can't access the surface, you can force the arm to move despite
   the brakes.  This is not something you should do too often, but it
   can help during the setup: `YouTube video <https://www.youtube.com/shorts/wBXQduLbHdE>`_.

To program the ESPM, connect an :ref:`ESXX programmer <esxx>` to the JTAG port on the ESPM board. Once programmed, the ESXX programmer should be removed and the cover reinstalled.


.. _essj:

ESSJ
====

Introduction
************

The circuit board (ESSJ) inside the patient cart manages the setup joint
potentiometers for the Si SUJ.

Like the ESPM, it communicates by default using a proprietary protocol over LVDS. JHU developed a custom FPGA firmware for it using an open communication protocol.

.. figure:: /images/Si/ESPM-ESSJ-SUJ.jpg
   :width: 400
   :align: center

   Patient cart SUJ setup with 4 ESPMs and 4 ESSJs

To program the ESSJ, connect an :ref:`ESXX programmer <esxx>` to the JTAG port on the ESSJ board. Once programmed, the ESXX programmer should be removed.

.. _esxx:

ESXX Programmer
===============

Introduction
************

The ESXX programmer replaces the obsolete ESPM programmer. It is a **one-time use** tool: it is needed to replace the vendor's firmware (from Intuitive) on the ESPM and ESSJ boards with custom dVRK firmware using an open communication protocol over LVDS. Once the boards have been reprogrammed, the ESXX programmer should be removed.

The ESXX programmer uses a micro SD card containing the custom firmware images (see :ref:`dvrk-sd-card-updater <sd-card-updater>`).

Video tutorial: `YouTube video <https://youtu.be/yIdvk0Wox8w>`_

JTAG Connections
****************

To connect the ESXX programmer to either an ESPM or ESSJ board:

1. Identify the JTAG connector on the target board:

   * On the **ESPM** (located in the arm), connect to the **J22 JTAG** port.
   * On the **ESSJ** (located in the SUJ), connect to the **JTAG** port.

.. figure:: /images/Si/ESPM-jtag-port.jpg
   :width: 400
   :align: center

   JTAG port on the ESPM board

.. figure:: /images/Si/ESJJ-jtag-port.jpg
   :width: 400
   :align: center

   JTAG port on the ESSJ board

2. Connect the cable between the target board and the ESXX programmer.

.. warning::

   Do not force connectors! Check the direction first. The yellow marker on the cable should be on the **ESXX Programmer** side!

.. figure:: /images/Si/ESXX-programmer-connectors.jpg
   :width: 400
   :align: center

   Connecting the ESXX programmer cable

Testing
*******

Once programmed and after disconnecting the ESXX programmer, the ESPM is powered through the dVRK Si controller.

When you power the dVRK controller, its front LEDs (PL,
PS, ESPM, COM, 48V and AMP) will flash green from left to right until
the firmware is found on the SD card.  Once the dVRK controller has
booted, the PL LED should blink green.  The ESPM LED should be solid
green unless there is an issue between the dVRK
controller and the ESPM.

The custom ESPM firmware will trigger the two LEDs on the arm itself (by the sterile adapter) to
blink a pinkish light back and forth.  This confirms that the arm has booted using the dVRK firmware.
