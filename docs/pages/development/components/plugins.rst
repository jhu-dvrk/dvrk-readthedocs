.. include:: /includes/logic-view-console.rst


Plugins
#######

.. _components-alternative-hardware:

Alternative hardware
====================

The dVRK system also supports generic arms, i.e. one can use a
different type of hardware as long as there is a cisst/SAW component
for it and the component has a *provided interface* that matches the
dVRK arm it is meant to replace.  We've successfully integrated some
alternate for the MTM:

* Sensible Phantom Omni using *sawSensablePhantom*
  (https://github.com/jhu-saw/sawSensablePhantom): The Omni doesn't
  provide a gripper so when used with the default dVRK teleoperation,
  the jaws are ignored.  The two buttons on the stylus can be used to
  emulate the dVRK foot pedals for "operator present" and "clutch".
* ForceDimension haptic devices and Novint Falcon using
  *sawForceDimensionSDK*
  (https://github.com/jhu-saw/sawForceDimensionSDK): The
  ForceDimension devices offer different features based on the model.
  We only tested models with 7 degrees of freedom, i.e. position,
  orientation and gripper.  If the orientation is motorized, it can be
  used like a da Vinci MTM, and we can enforce that the MTM orientation
  matches the PSM orientation.  ForceDimension devices don't have
  buttons so we either have to use a USB foot pedal or the GUI
  for "operator present" and "clutch".  The Novint Falcon is not as
  useful for real applications since it doesn't have a wrist, but it can
  be used for simple demos and debugging.  See example of
  configuration file
  ``jhu-dVRK-Si-demo/system-Novint-Falcon-PSM1-Teleop.json``
  (https://github.com/dvrk-config/dvrk_config_jhu).

.. figure:: /images/software/dVRK-component-ForceDimension.*
   :width: 400
   :align: center

   Using an alternate component
