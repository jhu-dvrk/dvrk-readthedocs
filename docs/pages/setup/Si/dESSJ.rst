dESSJ
=====

Introduction
************

The circuit board (ESPM) inside the arm manages all the low power
signals for the Si PSMs and ECMs: encoders, magnetic potentiometers,
buttons, instrument Id, LEDs.

By default, it communicates with the core controller using a
proprietary protocol over LVDS.  JHU, in collaboration with Intuitive
Surgical, developed a closed-source FPGA firmware that relies an open
protocol. The firmware is publicly available in a binary image only,
its source is and will remain private.

The ESPM programmer acts as an alternative boot loader and re-programs
the arm every time it powers up. The dVRK specific firmware is not
persistent and the arm will revert to the original firmware after a
power cycle if the ESPM programmer is removed or de-activated (using
switch on ESPM programmer).  The ESPM programmer uses an SD card so we
can easily upgrade the dVRK firmware.

Installation
************

You first need to make sure the arm is not power.

Then remove the plastic cover on the robot arm. You will need an
imperial allen wrench to remove the single bolt holding the cover.
You will then need to wiggle or pry-out the cover since it's also held
by a few clips.

.. figure:: /images/Si/ESPM-cover.jpg
   :width: 400
   :align: center

   ESPM cover

Connect the ESPM programmer pigtail cable to J22 JTAG on ESPM. There
are multiple identical connectors on the board. Make sure you
positively identify the connector on the ESPM by the label ("J22
JTAG") before you plug the cable in. The connector is keyed and please
do not force it upside down.

.. caution:

   The cable between the ESPM and the ESPM programmer has identical
   connectors on both ends but plugging it in backward will not
   work. If you are confused, look closely at the picture to see which
   pins are populated (i.e. have a black wire crimped)
   
.. figure:: /images/Si/ESPM-JTAG.jpg
   :width: 400
   :align: center

   ESPM JTAG and ESPM programmer

To hold the ESPM programmer to the arm, you should have received a
rubber credit card holder with double sided tape.  Place the ESPM
programmer in the holder, then stick the holder on the robot as shown
below. Put the plastic cover back on the arm while making sure the
cover is not pinching the cable.

.. note:

   If the arm is folded and you can access the surface to stick the
   holder, you can let him hand until you can power the arm and
   release the brakes.

Usage
*****

Make sure that the switch on the ESPM programmer is set to :enable", and that the micro SD is present. 

document LED on programmer
