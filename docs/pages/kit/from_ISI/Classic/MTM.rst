.. _isi_mtms:

Master Tele Manipulators
########################

There are two MTMs per system, one for the left hand and one for the
right hand.  The left and right MTMs are identical except for the
wrist gimbal assembly which are symmetrical.

.. figure:: /images/Classic/MTM/MTM-Classic-white-background.jpeg
   :width: 400
   :align: center

   da Vinci Classic and S MTM

Basic information:

* 7 motorized degrees of freedom, 7 actuators with optical encoders
* Some joints (second to fourth) are controlled by multiple actuators
  (actuator to joint coupling matrix)
* All joints have analog potentiometers, used for both homing and
  safety checks
* The gripper has a position sensor (Hall effect) but is not motorized
* The arm is designed with counter weights to stay as balanced as possible
* Motors are used to:

  * Fine tune the gravity compensation
  * Enforce the wrist orientation to match the PSM's orientation
    (:ref:`teleoperation <teleoperation>`).
  * Provide some force feedback to reflect mechanical limits on the
    patient's side
* The 7th degree of freedom allows the wrist platform to move around
  and provides a large rotational space (SO3)
* The gripper uses a combination of passive springs to provide some
  feedback

