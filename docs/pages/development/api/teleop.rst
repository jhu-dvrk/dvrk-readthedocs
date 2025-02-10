Teleoperation
#############

PSM
***

C++ class is ``mtsTeleOperationPSM``.  Tele-operation components names
are typically all upper case use the name of the MTM and PSM (e.g. for
the cisst component: ``MTMR-PSM1``).  Topics for ROS are published
under the namespace ``MTMx_PSMx`` (e.g. ``MTML_PSM1``,
``MTMR_PSM3``...).  Note that the ``-`` is replaced by ``_`` as ROS
doesn't support the minus character in namespaces and topics.

* ``current_state``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: current state.  Possible values are defined in
    ``components/code/mtsTeleOperationPSM.cpp``: ``DISABLED``,
    ``SETTING_ARMS_STATE``, ``ALIGNING_MTM`` and ``ENABLED``.

* ``desired_state``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs/String``
  * dVRK specific: desired state.  Possible values are defined in
    ``components/code/mtsTeleOperationPSM.cpp``: ``DISABLED``,
    ``ALIGNING_MTM`` and ``ENABLED``.

* ``state_command``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``crtk_msgs/StringStamped``
  * dVRK specific: send command to change desired state.  Possible
    values are: ``enable``, ``disable`` and ``align_mtm``

* ``following``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the PSM is following the MTM.  This can
    only happen if the tele-operation is ``ENABLED``, the user has
    engaged the MTM and the tele-operation is not clutched.  This can
    can be used to detect when the tele-operation component is
    actually sending commands to the PSM (using a combination of
    ``servo_cp`` and ``jaw/servo_jp``).

* ``scale``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs/Float64``
  * dVRK specific: indicate what is the current scaling factor between
    the MTM and PSM translations.

* ``set_scale``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs/Float64``
  * dVRK specific: set the scaling factor between the MTM and PSM
    translations.  This command changes the scale for this
    tele-operation component only.  **Use with caution**, it might be
    confusing for a user if both hands are not using the same scale.
    User should most likely use the ``console/teleop/set_scale``
    command instead.  This setting can also be changed using the GUI.

* ``align_mtm``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs/Bool``
  * dVRK specific: indicate if the tele-operation component will
    attempt to align the MTM orientation (with respect to the stereo
    display) to the orientation of the PSM end effector (with respect
    to the camera).  See ``set_align_mtm``.

* ``alignment_offset``

  * *cisst*: read command ``vctMatRot3``
  * *ROS*: publisher ``geometry_msgs/QuaternionStamped``
  * dVRK specific: difference between the MTM orientation and PSM
    orientation.  When ``align_mtm`` is set, the difference is capped
    by the maximum threshold allowed to engage (i.e. start following
    mode).  The default threshold is defined in
    ``components/include/sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h``
    and is set to 5 degrees.  When ``align_mtm`` is set to ``false``,
    this allows to track the difference of orientation between MTM and
    PSM when the operator starts tele-operating.

* ``set_align_mtm``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: set wether the tele-operation component requires
    the MTM orientation to match the PSM orientation to start the
    following mode.  When set, the tele-operation component will
    attempt to orient the MTM to match the PSM orientation.  For
    alternate MTMs without motorized wrist, the operator will have to
    manually re-orient the MTM to match the PSM orientation.  Also
    when set, in clutch mode, the component will lock the MTM
    orientation and leave the position (x, y, z) free so the operator
    can re-position their hands while preserving the orientation.  By
    default ``align_mtm`` is set to ``true`` and it mimics the
    behavior of the clinical da Vinci systems.  Setting ``align_mtm``
    to false allows relative orientation between the MTM and the PSM.
    This can be useful for alternate MTMs with a smaller SO3 space
    (e.g. ForceDimension haptic systems or Phanton Omni).  This
    setting can also be changed using the GUI.

* ``rotation_locked``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``sensor_msgs/Joy``
  * dVRK specific: indicate if the rotation is locked.  See
    ``lock_rotation``.

* ``lock_rotation``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: lock the orientation.  On the PSM side, the
    tele-operation component will only send translation commands and
    will not change the orientation of the tool tip.  On the MTM side,
    the component will lock the wrist (similar to clutch in following
    mode when ``align-mtm`` is set).  This setting can also be changed
    using the GUI.

* ``translation_locked``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``sensor_msgs/Joy`` Triggers power off sequence for the whole system.
  * dVRK specific: indicate if the translation is locked.  See
    ``lock_translation``.

* ``lock_translation``

  * *cisst*: write command ``bool``
  * *ROS*: subscriber ``std_msgs/Bool``
  * dVRK specific: lock the translation.  On the PSM side, the
    tele-operation component will only send rotation commands and will
    not change the position of the tool tip.  There is no effect on
    the MTM side.  This setting can also be changed using the GUI.

ECM
***

C++ class is ``mtsTeleOperationECM``.

* ``current_state``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs::String``
  * dVRK specific: see similar command for PSM tele-operation.

* ``desired_state``

  * *cisst*: event write ``std::string``
  * *ROS*: publisher ``std_msgs::String``
  * dVRK specific: see similar command for PSM tele-operation.

* ``state_command``

  * *cisst*: write command ``std::string``
  * *ROS*: subscriber ``std_msgs::String``
  * dVRK specific: see similar command for PSM tele-operation.

* ``following``

  * *cisst*: event write ``bool``
  * *ROS*: publisher ``std_msgs::Bool``
  * dVRK specific: see similar command for PSM tele-operation.

* ``scale``

  * *cisst*: event write ``double``
  * *ROS*: publisher ``std_msgs::Float64``
  * dVRK specific: see similar command for PSM tele-operation.

* ``set_scale``

  * *cisst*: write command ``double``
  * *ROS*: subscriber ``std_msgs::Float64``
  * dVRK specific: see similar command for PSM tele-operation.
