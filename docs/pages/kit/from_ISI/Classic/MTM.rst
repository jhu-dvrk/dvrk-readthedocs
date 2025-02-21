.. _mtm:

.. include:: /includes/logic-view-classic-arm.rst

Master Tele Manipulators
########################

There are two MTMs per system, one for the left hand and one for the
right hand.  The left and right MTMs are identical except for the
wrist gimbal assemblies which are mirrored.

The MTMs are used to track the position of the surgeon's hand and
provide some force feedback (i.e. they are haptic devices).

.. figure:: /images/Classic/MTM/MTM-Classic-white-background.jpeg
   :width: 400
   :align: center

   da Vinci Classic and S MTM

Basic information:

* Each MTM is attached to the surgeon console using 4 bolts.  The first
  axis (rotation) should be vertical.
* 7 motorized degrees of freedom, 7 actuators with optical encoders.
  MTMs are over-actuated
* The first 4 motors are located on the shoulder and use cable
  transmission, the last 3 are on the wrist and use gears
* Some joints (second to fourth) are controlled by multiple actuators
  (actuator to joint coupling matrix)
* All joints have analog potentiometers, used for both homing and
  safety checks
* The gripper has an analog position sensor (Hall effect) and is not
  motorized
* The arm is designed with counter weights to stay as balanced as
  possible
* Motors are used to:

  * Fine tune the gravity compensation
  * Enforce the wrist orientation to match the PSM's orientation
    (:ref:`teleoperation <teleop-psm>`).
  * Provide some force feedback to reflect mechanical limits on the
    patient's side
* The 7th degree of freedom allows the wrist platform to move around
  and provides a large rotational space (SO3)
* The gripper uses a combination of passive springs to emulate
  grasping forces
