Console
#######

C++ class is ``mtsIntuitiveResearchKitConsole``.

General
*******

* ``console/power_off``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: trigger power off sequence for the whole system.

* ``console/power_on``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: trigger power on sequence for the whole system.  If
    some arms are in ``FAULT`` state, this method will first
    ``disable`` them.

* ``console/home``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: triggers homing procedure for the whole system,
    including powering if the system is not yet powered.  Note that
    "homing" means findind where the arm is.  It doesn't always imply
    moving to the zero position.

* ``console/camera``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the console assumes it should
    tele-operate the ECM if all conditions are met: operator is
    present and tele-operation is enabled.  When in camera mode, the
    console disables the PSM tele-operation components (if any) and
    enables the ECM tele-operation component (if present).

* ``console/clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the console assumes it should clutch
    the active tele-operation components.  The console component
    simple passes the clutch state to the tele-operation components
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

Tele-operation
**************

* ``console/teleop/enabled``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the tele-operation is enabled at the
    console level.

* ``console/teleop/enable``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: enable or disable the tele-operation at the console
    level.  When tele-operation is enabled for the console, the
    console will manage with tele-operation components should be
    enabled using the following logic:

    * Tele-operation components must be declared in the console JSON
      configuration file: ``MTMR-PSM1``, ``MTML-PSM2``, ``MTMR-PSM3``,
      ``MTML-MTMR-ECM``...
    * For PSM tele-operation, the pair (e.g. ``MTMR-PSM1``) also needs
      to be selected (there should be one pair per MTM selected by
      default when the console starts).
    * Finally the console determines which type of tele-operation
      component should be enabled based on the *camera* input:

      * If the *camera* input is off (i.e. *camera* foot pedal not
        pressed), the console will enable the PSM tele-operation
        components that have been selected.
      * If the *camera* input is on, the console will enable the ECM
        tele-operation component (if declared in the console
        configuration file)

    * When a tele-operation is enabled, it will perform its own logic
      before getting into ``following`` mode...

* ``console/teleop/scale``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs::Float64``
  * dVRK specific: last scale sent to all tele-operation components.
    If a scale is set directly for a specific tele-operation component
    (i.e. not using the console tele-operation scale), said component
    can potentially use a different scale from the others.

* ``console/teleop/set_scale``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set the scale for all the tele-operation components
    declared in the console configuration file.

* ``console/teleop/teleop_psm_selected``

  * *cisst*: event write ``prmKeyValue``
  * *ROS*: publisher ``diagnostic_msgs/KeyValue``
  * dVRK specific: indicate which pairs of MTM-PSMs are currently
    selected (PSM tele-operation components).

* ``console/teleop/teleop_psm_unselected``

  * *cisst*: event write ``prmKeyValue``
  * *ROS*: publisher ``diagnostic_msgs/KeyValue``
  * dVRK specific: indicate which pairs of MTM-PSMs are currently
    unselected (PSM tele-operation components).

* ``console/teleop/cycle_teleop_psm_by_mtm``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs/String``
  * dVRK specific: cycle PSM tele-operation for a given MTM.  For
    example, if the console has the pairs ``MTML-PSM2`` and
    ``MTML-PSM3`` and ``MTML-PSM2`` is currently selected, using
    ``cycle_teleop_by_mtm(MTML)`` will unselect ``MTML-PSM2`` and
    select ``MTML-PSM3``.  There is a special case hard-coded in the
    console code to mimic the behavior of a clinical da Vinci system.
    A quick-tap on the clutch pedal will trigger a
    ``cycle_teleop_psm_by_mtm`` for the MTM that has been used for two
    PSM tele-operations declared in the console configuration file.

* ``console/teleop/select_teleop_psm``

  * *cisst*: write command ``prmKeyValue``
  * *ROS*: subscriber ``diagnostic_msgs/KeyValue``
  * dVRK specific: select a specific MTM-PSM tele-operation.  The
    KeyValue message allows to send two strings, i.e. the names of the
    MTM and PSM for the tele-operation component to select.  If the
    MTM is currently associated to a selected pair, said pair will
    first be unselected.  If the second value of the message is an
    empty string, the command deactivates the tele-operation currently
    using the MTM (first value in message)

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
