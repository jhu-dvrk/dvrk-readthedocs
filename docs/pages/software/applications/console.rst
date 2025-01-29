.. _console:

Console
#######

Overview
********

The dVRK console application comes with or without ROS.  For ROS, it's
the node ``dvrk_console_json`` from the package ``dvrk_robot``.

::

   ros2 run dvrk_robot dvrk_console_json

The non-ROS version is an executable found in the dVRK path.

::

   sawIntuitiveResearchKitQtConsoleJSON

Command line options
********************

Options for ROS and non-ROS console applications

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

ROS options
===========

::
   
 -j <value>, --json-config <value> : json configuration file (required)
 -p <value>, --ros-period <value> : period in seconds to read all arms/teleop components and publish (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the arm component's period (optional)
 -P <value>, --tf-ros-period <value> : period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period (optional)
 -i, --ros-io-config : json config file to configure ROS bridges to collect low level data (IO) (optional)
 -s, --suj-voltages : add ROS topics for SUJ voltages (optional)
 -I, --pid-topics : add some extra publishers to monitor PID state (optional)
 -t, --text-only : text only interface, do not create Qt widgets (optional)
 -c <value>, --collection-config <value> : json configuration file for data collection using cisstMultiTask state table collector (optional)
 -C, --calibration-mode : run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing.  This mode should only be used when calibrating your potentiometers (optional)
 -m, --component-manager : JSON files to configure component manager (optional)
 -S <value>, --qt-style <value> : Qt style, use this option with a random name to see available styles (optional)
 -D, --dark-mode : replaces the default Qt palette with darker colors (optional)


Configuration
*************

Using component manager configuration files
