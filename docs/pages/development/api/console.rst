.. _api-system:

System
######

C++ class is ``dvrk::system``.

General
*******

* ``system/power_off``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: trigger power off sequence for the whole system.

* ``system/power_on``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: trigger power on sequence for the whole system.  If
    some arms are in ``FAULT`` state, this method will first
    ``disable`` them.

* ``system/home``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: triggers homing procedure for the whole system, including
    powering if the system is not yet powered.  Note that "homing" means finding
    where the arm is (absolute position).  It doesn't always imply moving to the
    zero position.

Per console
***********

One can have multiple consoles, in this case the namespace can be ``console1/``,
``console2/``... instead of ``console/``. * ``console/camera``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the console assumes it should
    teleoperate the ECM if all conditions are met: operator is
    present and teleoperation is enabled.  When in camera mode, the
    console disables the PSM teleoperation components (if any) and
    enables the ECM teleoperation component (if present).

* ``console/clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the console assumes it should clutch
    the active teleoperation components.  The console component
    simple passes the clutch state to the teleoperation components
    who then have to handle the clutch.

* ``console/operator_present``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the console assumes the operator is
    present (either through a dead man switch/foot pedal, head sensor
    or emulated).

* ``console/emulate_camera``

  * *cisst*: write command ``prmEventButton``
  * *ROS*: subscriber ``sensor_msgs/Joy``
  * dVRK specific: emulate the camera pedal press.

* ``console/emulate_clutch``

  * *cisst*: write command ``prmEventButton``
  * *ROS*: subscriber ``sensor_msgs/Joy``
  * dVRK specific: emulate the clutch pedal press.

* ``console/emulate_operator_present``

  * *cisst*: write command ``prmEventButton``
  * *ROS*: subscriber ``sensor_msgs/Joy``
  * dVRK specific: emulate the operator presence sensor.

* ``console/teleop/enabled``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the teleoperation is enabled at the
    console level.

* ``console/teleop/enable``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: enable or disable the teleoperation at the console
    level.  When teleoperation is enabled for the console, the
    console will manage with teleoperation components should be
    enabled using the following logic:

    * Teleoperation components must be declared in the console JSON
      configuration file: ``MTMR_PSM1``, ``MTML_PSM2``, ``MTMR_PSM3``,
      ``MTML_MTMR_ECM``...
    * For PSM teleoperation, the pair (e.g. ``MTMR_PSM1``) also needs
      to be selected (there should be one pair per MTM selected by
      default when the console starts).
    * Finally, the console determines which type of teleoperation
      component should be enabled based on the *camera* input:

      * If the *camera* input is off (i.e. *camera* foot pedal not
        pressed), the console will enable the PSM teleoperation
        components that have been selected.
      * If the *camera* input is on, the console will enable the ECM
        teleoperation component (if declared in the console
        configuration file)

    * When a teleoperation is enabled, it will perform its own logic
      before getting into ``following`` mode...

* ``console/teleop/scale``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs::Float64``
  * dVRK specific: last scale sent to all teleoperation components.
    If a scale is set directly for a specific teleoperation component
    (i.e. not using the console teleoperation scale), said component
    can potentially use a different scale from the others.

* ``console/teleop/set_scale``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set the scale for all the teleoperation components
    declared in the system configuration file.

* ``console/teleop/teleop_selected``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs::String``
  * dVRK specific: indicate which pairs of MTMs/PSMs are currently
    selected (PSM and ECM teleoperation components).

* ``console/teleop/teleop_unselected``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs::String``
  * dVRK specific: indicate which pairs of MTMs/PSMs are currently
    unselected (PSM and ECM teleoperation components).

* ``console/teleop/select_teleop``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs::String``
  * dVRK specific: select a specific PSM or ECM teleoperation.  The string
    message is the name of the teleoperation component to select (``MTMR_PSM1``,
    ``MTMR1_MTML1_ECM``...).  If there is any conflict between the newly
    selected teleoperation and a previously selected teleoperation, the
    previously selected teleoperation is automatically unselected.  A conflict
    is defined by any arm (surgeon's or patient's side) used by both
    teleoperation components.

* ``console/teleop/unselect_teleop``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs::String``
  * dVRK specific: unselect a specific PSM or ECM teleoperation.

Foot pedals
***********

* ``footpedals/clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *clutch* is released
    (``0``), pressed (``1``) or a quick tap happened (``2``).

* ``footpedals/camera``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *camera* is released
    (``0``), pressed (``1``) or a quick tap happened (``2``).

* ``footpedals/cam_minus``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *camera -* is
    released (``0``), pressed (``1``) or a quick tap happened (``2``).

* ``footpedals/cam_plus``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *camera +* is
    released (``0``), pressed (``1``) or a quick tap happened (``2``).

* ``footpedals/bicoag``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *bicoag* is released
    (``0``), pressed (``1``) or a quick tap happened (``2``).

* ``footpedals/coag``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the physical pedal *coag* is released
    (``0``), pressed (``1``) or a quick tap happened (``2``).
