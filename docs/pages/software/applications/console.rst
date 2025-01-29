.. _console:

Console
#######

Overview
********

The dVRK console application is the main way to interact with a dVRK
system. It comes in two versions:

* ``sawIntuitiveResearchKitQtConsoleJSON``

  * C+ application with a Qt based GUI, it depends on cisst/SAW
  * Compatible with all dVRK controllers (including for Windows/MacOS over UDP)
  * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications

* ``dvrk_robot dvrk_console_json``

  * C+ application with a Qt based GUI and ROS interface, it depends on cisst/SAW
* Compatible with all dVRK controllers
  * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/ros/dvrk_robot

The dVRK console application comes with or without ROS.  For ROS, it's
the node ``dvrk_console_json`` from the package ``dvrk_robot``.

::

   ros2 run dvrk_robot dvrk_console_json

The non-ROS version is an executable found in the dVRK path.

::

   sawIntuitiveResearchKitQtConsoleJSON


Command line options
********************

``dvrk_console_json`` supports all the command line options of
``sawIntuitiveResearchKitQtConsoleJSON`` as well as options to
configure the ROS publish rates and some optional ROS topics.

Shared options
==============

::

 -j <value>, --json-config <value> : json configuration file (required)
 -t, --text-only : text only interface, do not create Qt widgets (optional)
 -c <value>, --collection-config <value> : json configuration file for data collection using cisstMultiTask state table collector (optional)
 -C, --calibration-mode : run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing.  This mode should only be used when calibrating your potentiometers. (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -S <value>, --qt-style <value> : Qt style, use this option with a random name to see available styles (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)

ROS extra options
=================

::

 -p <value>, --ros-period <value> : period in seconds to read all arms/teleop components and publish (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the arm component's period (optional)
 -P <value>, --tf-ros-period <value> : period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period (optional)
 -i, --ros-io-config : json config file to configure ROS bridges to collect low level data (IO) (optional)
 -s, --suj-voltages : add ROS topics for SUJ voltages (optional)
 -I, --pid-topics : add some extra publishers to monitor PID state (optional)


Configuration
*************

Using component manager configuration files, links to architecture bridges.
