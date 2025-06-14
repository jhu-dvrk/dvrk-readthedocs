Introduction
############

Each component of the dVRK described in the :ref:`software
architecture <architecture>` provides a set of functionalities,
i.e. commands/events for a |cisstMultiTask|_ component or
topics/services for a ROS node (see :ref:`ROS bridges <bridge-ros>`)

In general, we try to expose most C++ commands and events as ROS
topics or services under the same name.  Starting with the dVRK
release 2.0, we are using the |CRTK|_ naming convention.  There are
also some commands very specific to the dVRK not covered by CRTK.  ROS
bridge for the dVRK specific commands can be found in
``dvrk_system.cpp``
(https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/ros/dvrk_robot/src/dvrk_system.cpp)

For the CRTK commands, the cisst to ROS CRTK bridge
(https://github.com/jhu-cisst/cisst-ros/tree/main/cisst_ros_crtk) is
used.

If you are migrating your *cisstMultiTask* or ROS code from the dVRK
1.7, you can find some porting information in the directory
``crtk-port``
(https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/crtk-port).
