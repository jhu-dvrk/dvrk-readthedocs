.. _height:

Console height adjustement
**************************

Introduction
============

On the da Vinci Classic (aka Standard) and S, the HRSV (High Resolution
Stereo Video) can be moved up and down to adjust to the height of the
user.  Some dVRK groups have received the original surgeon console,
sometimes as-is and some other times gutted from their electronics.

Original electronics
====================

If you still have the original electronics, you can simply plug the
da Vinci console to a power outlet and power it.  Then you can use the
two black buttons somewhat hidden on the left side of the arm rest to
raise and lower the top part of the console.

.. figure:: /images/Classic/console-lift-original-labeled.jpg
   :width: 600
   :align: center

   Classic console original lift

dVRK solution
=============

If you don't have the original electronics, you can hack together a
simple motor controller.  For this you will need:

* A power supply, ideally providing 24V DC.  You can use an old laptop
  power supply.
* A PWM motor controller, ideally adjustable, so you can drive the lift
  up AND down.  We used the following from Amazo,n but there are tons
  of alternatives:
  https://www.amazon.com/EPLZON-Controller-Adjustable-Electric-Regulator/dp/B0BVBKDY1Q/

First connect your power supply output to the input of the PWM motor
controller.  Then connect the output of the motor controller to the
motor lift.  You can find the motor under the HRSV.  The motor cables
go towards the back of the surgeon's console using a pass-through.
At JHU, we just zip-tied the controller to the side of the console.

.. figure:: /images/Classic/console-lift-dVRK-labeled.jpg
   :width: 600
   :align: center

   Classic console dVRK lift
