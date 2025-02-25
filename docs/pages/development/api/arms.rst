Arms
####

C++ class is ``mtsIntuitiveResearchKitArm``.  Arm names are typically
all upper case and follow the da Vinci naming convention: MTML and
MTMR for MTM left and right, PSM1, PSM2 and PSM3 for PSMs and finally
ECM.  ROS topics are organized in a namespace using the arm's name
(e.g. ``MTMR/setpoint_cp``).

All
***

Operating state
===============

* ``state_command``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``crtk_msgs/StringStamped``
  * |CRTK|_: send an operating state command

* ``operating_state``

  * *cisst*: write event and read command ``prmOperatingState``
  * *ROS*: publisher ``crtk_msgs/operating_state``
  * |CRTK|_: current operating state

* ``desired_state``

  * *cisst*: write event ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: uses the CRTK state whenever possible

* ``error``

  * *cisst*: write event ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: error messages, can be used for custom GUI.  For
    ROS, these messages are also sent as errors for ROS log.

* ``warning``

  * *cisst*: write event ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: warning messages, can be used for custom GUI.  For
    ROS, these messages are also sent as warnings for ROS log.

* ``status``

  * *cisst*: write event ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: status messages, can be used for custom GUI.  For
    ROS, these messages are also sent as status messages for ROS log.

* ``goal_reached``

  * *cisst*: write event ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: boolean that indicates if the last
    ``move_{jc}{pr}`` command was completed successfully or not.  It
    is possible to use the CRTK ``operating_state`` fields ``is_busy``
    and ``state`` instead.  This is provided for backward
    compatibility with dVRK 1.x applications.

Motion queries
==============

* ``measured_cp``

  * *cisst*: read command ``prmPositionCartesianGet``
  * *ROS*: publisher ``geometry_msgs/TransformStamped``
  * |CRTK|_: measured cartesian position.https://github.com/jhu-cisst/cisst-ros/tree/main/cisst_ros_crtk

* ``measured_cv``

  * *cisst*: read command ``prmVelocityCartesianGet``
  * *ROS*: publisher ``geometry_msgs/TwistStamped``
  * |CRTK|_: measured cartesian velocity.

* ``measured_js``

  * *cisst*: read command ``prmStateJoint``
  * *ROS*: publisher ``sensor_msgs/JointState``
  * |CRTK|_: measured joint state (position, velocity, effort).

* ``setpoint_cp``

  * *cisst*: read command ``prmPositionCartesianGet``
  * *ROS*: publisher ``geometry_msgs/TransformStamped``
  * |CRTK|_: cartesian position setpoint.

* ``setpoint_js``

  * *cisst*: read command ``prmStateJoint``
  * *ROS*: publisher ``sensor_msgs/JointState``
  * |CRTK|_: joint setpoint (either position or effort).

* ``local/measured_cp``

  * *cisst*: read command ``prmPositionCartesianGet``
  * *ROS*: publisher ``geometry_msgs/TransformStamped``
  * dVRK specific: measured cartesian position relative to the first
    frame of the kinematic chain.  This doesn't include any base
    frame.  For the PSMs and ECM, the first frame of the kinematic
    chain is centered on the RCM point.  For the MTMs, the first frame
    of the kinematic chain is centered near the mounting point.  The
    non "local" ``measured_cp`` includes the base frame.  For example,
    MTM cartesian positions are defined with respect to the stereo
    display and the PSM cartesian positions are defined with respect
    to the endoscope (i.e. ECM tip).  See also :ref:`dVRK coordinate
    systems <devel-frames>`.

* ``local/setpoint_cp``

  * *cisst*: read command ``prmPositionCartesianGet``
  * *ROS*: publisher ``geometry_msgs/TransformStamped``
  * dVRK specific: cartesian setpoint relative to the first frame of
    the kinematic chain (see notes for ``local/measured_cp``).https://github.com/jhu-cisst/cisst-ros/tree/main/cisst_ros_crtk

* ``body/jacobian``

  * *cisst*: read command ``vctDoubleMat``
  * *ROS*: publisher ``std_msgs/Float64MultiArray``
  * dVRK specific: body jacobian, i.e. relative to end effector. See
    |cisstRobotManipulator|_.

* ``body/measured_cf``

  * *cisst*: read command ``prmForceCartesianGet``
  * *ROS*: publisher ``geometry_msgs/WrenchStamped``
  * dVRK specific. Estimated forces on the end effector. These are
    computed using the current feedback on the actuators. From there,
    the joint efforts are estimated using the actuator to joint
    coupling matrix. See also ``body/set_cf_orientation_absolute`` and
    |cisstRobotManipulator|_. Finally, the cartesian effort is
    computed using the jacobian. This is a rough cartesian force
    emulation as the computations don't take into account gravity
    compensation nor any other dynamic model or the arm.

* ``spatial/jacobian``

  * *cisst*: read command ``vctDoubleMat``
  * *ROS*: publisher ``std_msgs/Float64MultiArray``
  * dVRK specific: spatial jacobian, i.e. relative to the base frame
    (first frame in kinematic chain). See |cisstRobotManipulator|_.

* ``spatial/measured_cf``

  * *cisst*: read command ``prmForceCartesianGet``
  * *ROS*: publisher ``geometry_msgs/WrenchStamped``
  * dVRK specific: see ``body/measured_cf``.

* ``forward_kinematics``:

  * *cisst*: qualified read command

    * ``vctDoubleVec``
    * ``vctFrm4x4``

  * *ROS*: service ``cisst_msgs/QueryForwardKinematics``

    * ``sensor_msgs/JointState jp``
    * ``geometry_msgs/PoseStamped cp``

  * dVRK specific: compute forward kinematic based on joint values
    provided.  The length of the vector of joint positions determines
    which frame should be computed along the kinematic chain.  For
    ROS, the field ``jp.position`` is used to store joint positions.
    This method prepends the base frame for the arm to the result.

* ``local/forward_kinematics``:

  * *cisst*: qualified read command

    * ``vctDoubleVec``
    * ``vctFrm4x4``

  * *ROS*: service ``cisst_msgs/QueryForwardKinematics``

    * ``sensor_msgs/JointState jp``
    * ``geometry_msgs/PoseStamped cp``

  * dVRK specific: compute forward kinematic based on joint values
    provided.  The length of the vector of joint positions determines
    which frame should be computed along the kinematic chain.  For
    ROS, the field ``jp.position`` is used to store joint positions.

Motion commands
===============

* ``servo_cp``

  * *cisst*: write command ``prmPositionCartesianSet``
  * *ROS*: subscriber ``geometry_msgs/TransformStamped``
  * |CRTK|_: set cartesian position goal for low-level controller
    (PID).  **Use with caution**, goals should be reachable within a
    single clock tick (< 1 ms).  Use ``move_cp`` for large motions.

* ``servo_jf``

  * *cisst*: write command ``prmForceTorqueJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: set joint effort goal for low-level controller (direct
    current control).  **Use with caution**, the only safety mechanism
    built in is the cap on maximum motor current.

* ``servo_jp``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: set joint position goal for low-level controller (PID).
    **Use with caution**, goals should be reachable within a single
    clock tick (< 1 ms).  Use ``move_jp`` or ``move_jr`` for large
    motions.

* ``servo_jr``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: set joint relative position goal for low-level controller
    (PID).  The goal is defined as an increment that will be added to
    the current ``setpoint_jp``.  See also notes for ``servo_jp``.

* ``spatial/servo_cf``

  * *cisst*: write command ``prmForceCartesianSet``
  * *ROS*: subscriber ``geometry_msgs/WrenchStamped``
  * dVRK specific: set cartesian effort goal for low-level controller
    using ``spatial/jacobian`` (direct current control).  **Use with
    caution**, the only safety mechanism built-in is the cap on
    maximum motor current.  For most application, use
    ``body/servo_cf``.  Gravity compensation will be added based on
    last call to ``use_gravity_compensation`` (for MTMs and ECM).  See
    |cisstRobotManipulator|_.

* ``body/servo_cf``

  * *cisst*: write command ``prmForceCartesianSet``
  * *ROS*: subscriber ``geometry_msgs/WrenchStamped``
  * dVRK specific: set cartesian effort goal for low-level controller
    using ``body/jacobian`` (direct current control).  **Use with
    caution**, the only safety mechanism built in is the cap on
    maximum motor current.  Useful for haptic on MTM.  By default
    direction of force is defined by the orientation of the end
    effector.  To use the absolute orientation, toggle on/off using
    ``body/set_cf_orientation_absolute``.  Gravity compensation will
    be added based on last call to ``use_gravity_compensation`` (for
    MTMs and ECM).  See |cisstRobotManipulator|_.

* ``set_cartesian_impedance_gains``

  * *cisst*: write command ``prmCartesianImpedanceGains``
  * *ROS*: subscriber ``cisst_msgs/prmCartesianImpedanceGains``
  * dVRK specific: apply wrench based on difference between measured
    and goal cartesian positions as well as twist (cartesian
    velocity).  The cartesian space is divided in 12 cases: negative
    and positive (**2**) error in position and orientation (**x 2**)
    along axes X, Y and Z (**x 3 = 12**).  The payload for this
    command includes 3 parameters for each case: a linear gain, a
    damping gain and an offset.  This command can be used to define a
    simple haptic virtual fixture (plane, line, point, box corner...).
    **Use with caution**, specially if the frame used to compute the
    cartesian impedance is far from the current arm position as this
    could lead to strong forces applied to the arm.  Internally the
    dVRK code uses the class ``osaCartesianImpedanceController`` from
    the package |sawControllers|_.

* ``move_cp``

  * *cisst*: write command ``prmPositionCartesianSet``
  * *ROS*: subscriber ``geometry_msgs/TransformStamped``
  * |CRTK|_: set cartesian trajectory goal.  The current
    implementation converts the cartesian goal into a joint trajectory
    goal and then execute the trajectory in joint space.  Therefore
    the current controller doesn't generate straight lines in
    cartesian space. See ``move_jp``.

* ``move_jp``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: set joint trajectory goal.

    * Goal can be changed before the trajectory is completed.  The
      trajectory generator will use the current joint velocities and
      accelerations to smooth the trajectory.
    * User can check if the trajectory is completed using ``is_busy``
      from the ``operating_state``.
    * The current implementation assumes the final velocity is zero.
      Sending a fast succession of goals can generate a stop-and-go
      motion.
    * Internally, the dVRK code uses the class ``robReflexxes`` from
      |cisstRobot|_ (wrapper for Reflexxes Type II Library:
      `<https://www.reflexxes.com>`_).

* ``move_jr``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: set relative joint trajectory goal.  See ``move_jp``.

Configuration
=============

* ``use_gravity_compensation``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: turn on or off gravity compensation. As of dVRK
    1.7, gravity compensation is well supported for MTMs. ECM gravity
    compensation has been introduced in dVRK 2.0 but is roughly tuned.
    It is used to help the low level controller (PID) and when the arm
    is in manual mode ("clutched").

* ``body/set_cf_orientation_absolute``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: when using ``body/servo_cf``, reference frame to
    apply the wrench is the end effector frame.  This makes sense for
    the position but can be confusing for the orientation.  For
    example, using the MTM, applying a contant force in Z direction
    feels like holding a rocket in your hand, the direction of the
    force will change as the user rotates the gripper.  To feel a
    force in a constant direction, independently of the hand's
    orientation, use ``set_cf_orientation_absolute``.

* ``trajectory_j/ratio``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs/Float64``
  * dVRK specific: ratio applied to both maximum velocity and
    acceleration used for joint trajectory generation.  If a user
    overrides the ratio using either the velocity or acceleration
    specific ratio, this value in undefined.  See
    ``trajectory_j/set_ratio``.

* ``trajectory_j/ratio_a``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs/Float64``
  * dVRK specific: atio applied to maximum acceleration used for joint
    trajectory generation.  See ``trajectory_j/set_ratio_a``.

* ``trajectory_j/ratio_v``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs/Float64``
  * dVRK specific: ratio applied to maximum velocity used for joint
    trajectory generation.  See ``trajectory_j/set_ratio_v``.

* ``trajectory_j/set_ratio``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set ratio applied to both maximum velocity and
    acceleration used for joint trajectory generation.  Ratio must be
    in range **]0, 1]**.  Default ratio is 1.  This is the recommended
    way to slow down arm trajectories.

* ``trajectory_j/set_ratio_a``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set ratio applied to maximum acceleration used for
    joint trajectory generation.  This is provided for backward
    compatibility and fine tuning but the recommended approach is to
    use ``trajectory_j/set_ratio``.

* ``trajectory_j/set_ratio_v``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set ratio applied to maximum velocity used for
    joint trajectory generation.  This is provided for backward
    compatibility and fine tuning but the recommended approach is to
    use ``trajectory_j/set_ratio``.

ECM
***

C++ class is ``mtsIntuitiveResearchKitArmECM``.

* ``manip_clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs::Joy``
  * dVRK specific: indicate if the clutch button on the ECM (located
    on top of the translation/insertion stage) is pressed or not.

* ``endoscope_type``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs::String``
  * dVRK specific: indicate which endoscope is currently in use.  Note
    that the endoscope type is not detected automatically so this
    setting depends on the user.  It can be modified using the GUI or
    programmatically.

* ``set_endoscope_type``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs::String``
  * dVRK specific: set the type of endoscope mounted on the ECM.  The
    endoscope type is used for two things.  Up/down/straight is used
    to compute the tool tip transformation for the forward kinematic.
    HD/SD is used for gravity compensation, the HD camera head happens
    to be a bit heavier than the SD one (see :ref:`video pipeline
    <video-sources>`).  Possible values are defined in file
    ``components/code/mtsIntuitiveResearchKitEndoscopeTypes.cdg``
    (see |cisstDataGenerator|_): ``NONE``, ``SD_STRAIGHT``, ``SD_UP``,
    ``SD_DOWN``, ``HD_STRAIGHT``, ``HD_UP``, ``HD_DOWN``.

MTM
***

C++ class is ``mtsIntuitiveResearchKitArmMTM``.

* ``gripper/measured_js``

  * *cisst*: read command ``prmStateJoint``
  * *ROS*: publisher ``sensor_msgs/JointState``
  * |CRTK|_: ``measured_js`` for the MTM gripper. The only field
    available is the position of the gripper.  These is no measurement
    available for velocity or effort.

* ``gripper/closed``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the gripper is closed or not based on a
    hard coded threshold (0.0).  This is provided for convenience and
    backward compatibility but users can instead use
    ``gripper/measured_js``, ``position[0]`` with their own threshold
    and logic to determine if the gripper is closed or not.

* ``gripper/pinch``

  * *cisst*: event void
  * *ROS*: publisher ``std_msgs/Empty``
  * dVRK specific: provided for backward compatibility.  Same as
    ``gripper/closed`` is ``true``.

* ``orientation_locked``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the orientation is locked or not.  See
    ``lock_orientation``.

* ``lock_orientation``

  * *cisst*: write command ``vctMatRot3``
  * *ROS*: subscriber ``geometry_msgs/Quaternion``
  * dVRK specific: send an orientation goal for the orientation of the
    MTM with respect to its base frame.  A joint trajectory is used to
    reach the orientation goal.  Once the MTM has reached the desired
    orientation, it will maintain said orientation even when the arm
    moves.  This command has no effect if the MTM is not controlled in
    effort mode (i.e. ``servo_cf``).  The best example of usage is to
    lock the MTM orientation (~wrist) when in clutch mode.  The
    operator can move around freely but the absolute orientation
    remains constant so the MTM is still aligned to the PSM when the
    user restart the tele-operation.

* ``unlock_orientation``

  * *cisst*: void command
  * *ROS*: subscriber ``std_msgs/Empty``
  * dVRK specific: free the orientation.  This is used when the
    operator ends the MTM to PSM clutch.

PSM
***

C++ class is ``mtsIntuitiveResearchKitArmPSM``.

* ``jaw/measured_js``

  * *cisst*: read command ``prmStateJoint``
  * *ROS*: publisher ``sensor_msgs/JointState``
  * |CRTK|_: ``measured_js`` for the PSM jaws. Position, velocity and
    effort are provided. Effort is based on the current feedback and
    can be affected by multiple factors so it is not an exact torque
    applied on the jaws.

* ``jaw/setpoint_js``

  * *cisst*: read command ``prmStateJoint``
  * *ROS*: publisher ``sensor_msgs/JointState``
  * |CRTK|_: ``setpoint_js`` for the PSM jaws.

* ``jaw/servo_jf``

  * *cisst*: write command ``prmForceTorqueJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: ``servo_jf`` for the PSM jaws.

* ``jaw/servo_jp``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: ``servo_jp`` for the PSM jaws.

* ``jaw/move_jp``

  * *cisst*: write command ``prmPositionJointSet``
  * *ROS*: subscriber ``sensor_msgs/JointState``
  * |CRTK|_: ``move_jp`` for the PSM jaws.

* ``tool_type``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: indicate which tool is currently in use.  Note that
    the tool type can be determined in different ways depending on
    your hardware and configuration files.  See :ref:`instrument
    detection<config-dallas>`..

* ``tool_type_request``

  * *cisst*: event void
  * *ROS*: publisher ``std_msgs/Empty``
  * dVRK specific: when using ``MANUAL`` :ref:`instrument detection
    <config-dallas>`, event that indicates that a new tool has
    been detected and the software needs to know which type of tool it
    is.  The tool type can also be set using the dropdown menu on the
    GUI PSM widget.

* ``set_tool_type``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs/String``
  * dVRK specific: when using ``MANUAL`` :ref:`instrument
    detection<config-dallas>`, set the tool type.  Possible
    values are defined in file
    ``components/code/mtsIntuitiveResearchKitToolTypes.cdg`` (see
    |cisstDataGenerator|_).  A tool description file with a filename
    matching the tool name needs to be provided as well.  Description
    files can be found in ``share/tool`` for many common da Vinci
    tools.  The tool type can also be set using the dropdown menu on
    the GUI PSM widget.

* ``set_adapter_present``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: tell the console that the sterile adapter is
    present without any actual hardware detection of the adapter.
    This can be used to force engaging a non-dVRK modified sterile
    adapter. **Use with caution**, this can lead to undesired motions
    if a tool is also inserted.  The vast majority of users should
    **not**, **ever** use this command.

* ``set_tool_present``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: tell the controller that a tool is present without
    any actual hardware detection of the tool.  This can be used to
    force engaging a tool without a Dallas chip. **Use with caution**,
    this can lead to undesired motions if the wrong tool is inserted.
    The vast majority of users should **not**, **ever** use this
    command.

* ``io/adapter``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific.  Indicate if the sterile adapter is present or not.

* ``io/tool``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific.  Indicate if a tool is present or not.

* ``io/manip_clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the manipulator clutch button is
    pressed or not.  This is the white button located on top of the
    translation stage on the PSM.  This button is used to release the
    PID on the arm and move it manually.

* ``io/suj_clutch``

  * *cisst*: event write ``prmEventButton``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the manipulator SUJ (Set Up Joints)
    clutch button is pressed or not.  This is the white button located
    on the side of the "horizontal" link of the PSM.  This button is
    used to release the brakes on the arm's SUJ if you happen to have
    the dVRK SUJ :ref:`controller<controller-versions>`.
