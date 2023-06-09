<!--ts-->
   * [Overview](#overview)
   * [Low level](#low-level)
      * [FPGA/QLA boards](#fpgaqla-boards)
      * [AmpIO library](#ampio-library)
   * [C  ](#c)
      * [Threads](#threads)
      * [Robot IOs](#robot-ios)
      * [PID controller](#pid-controller)
      * [Arm classes](#arm-classes)
      * [Tele-operation](#tele-operation)
      * [Console](#console)
   * [Qt widgets](#qt-widgets)
      * [Robot IOs](#robot-ios-1)
      * [PID controller](#pid-controller-1)
      * [Arm classes](#arm-classes-1)
      * [Tele-operation](#tele-operation-1)
      * [Console](#console-1)
   * [ROS](#ros)
      * [Topics](#topics)
      * [Python](#python)
      * [Matlab](#matlab)

<!-- Added by: anton, at: 2021-01-28T16:03-05:00 -->

<!--te-->

# Overview

The dVRK hardware and software stack is composed of:
* Firmware on FPGA/QLA interfacing IO with FireWire
* Lightweight C library on PC side to interface to FPGA via FireWire
* C++ components using the _cisst_/_SAW_ libraries to implement IOs, controllers (PID, tele-operation), console, GUI, bridges to ROS, ...
* ROS wrapper around dVRK topics

# Low level

## FPGA/QLA boards
Embedded firmware:
  * Collects data from digital inputs data (limit/home switches)
  * Controls digital outputs (ON/OFF/PWM)
  * Computes encoder positions and velocities, including detecting overflow and preload
  * Performs basic safety checks on motor current (consistency between requested and measured)
  * Implements subset of FireWire protocol to communicate with a computer
  * Maintains a watchdog to make sure the PC is still connected and communicating with the controller
  * See:
    * http://jhu-cisst.github.io/mechatronics
    * https://github.com/jhu-cisst/mechatronics-firmware

## AmpIO library
C low level library:
  * Runs on the PC sides on top of Linux/libraw1394
  * Packs/unpacks data to/from FPGA, i.e. convert bits to usable numbers (integers)
  * Handles multiple FPGA and treat them as single controller (
  * Simple text based programs to test hardware (`qladisp`, `qlatest`, `qlacommand`, ...)
  * See:
    * https://github.com/jhu-cisst/mechatronics-software/wiki
    * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Hardware

# C++

All C++ components are based on the _cisst_/_SAW_ libraries, more specifically the _cisstMultiTask_ framework:
* _cisst_ libraries: https://github.com/jhu-cisst/cisst/wiki
* _cisstMultiTask_: [tutorial](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-tutorial) and [concepts](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts)
* Overall architecture: [JHU LCSR 09/2021 internal meeting presentation](https://dvrk.lcsr.jhu.edu/downloads/presentations/LCSR-cisst-SAW-SoftwareInfrastructure-lab-meeting-09-2021.pptx)

The overall architecture is described in this picture:
![Overview](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dVRK-component-thread-view.png)

## Threads

In general, each component owns a thread and can run at its own frequency.  There are a few notable exceptions:
 * There is a single IO component with multiple interfaces (one per arm connected).  This is required to:
   * Minimize the number of IOs (inputs/outputs).  For n boards (2 per arm controllers in most cases), sequential reads and writes would require 2 x n IOs, one read and one write per board.  FireWire also allows to broadcast a single write command to multiple controllers and read from all controllers in a single message.  This reduces the number of IOs to a fixed number (~2), whatever the number of controllers are used.  This is only possible if a single thread manages all the IOs.
   * Avoid simultaneous accesses to the FireWire port from multiple threads (FireWire read/write are thread safe but processes can hang for a couple seconds).
 * The PID components could run in separate threads but this would introduce a fair amount of latency since the thread safe communication mechanisms in _cisstMultiTask_ are based on queues.   Assuming a 1 millisecond period for both IO and PID, the PID would read some cached data (position and velocity) from the IO (between 0+ and 1 millisecond old) and then request a new effort.  This request being queued will be acted on between 0+ and 1 millisecond later.  Overall, the time between read and write could be as high as 2 milliseconds.  Instead, we used the _cisstMultiTask_ ExecIn/ExecOut feature (see [cisstMultiTask concepts](https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts)) which allows to attach a component to another.  Effectively, the parent thread now runs the child's computation whenever needed.  In pseudo code:

```C++
  IO::Run(void) {
     ReadAllData();
     SaveReadDataInStateTable(); // state tables are used to cache data
     ExecOut(); // trigger PID.Run() for all PID components attached to this IO
     ProcessQueuedCommands(); // dequeue all commands, including those from PID
  }
```

 * Qt manages its own thread(s)
 * The ROS bridges (cisst-ros) can be configured based on the user's needs (see below)

## Robot IOs

The class `mtsRobotIO1394` is part of the [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) library.  It provides:
* conversion to from integer from SI units
* velocity estimation based on encoder counts and time since last change
* actuator/joint coupling for positions and efforts
* extra safety checks (consistency between potentiometers and encoders, compare required and measured current)
* configuration using XML, see `sawRobotIO*.xml` files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share/jhu-daVinci)
* support for motors and brakes (e.g. dVRK ECM)
* _cisstMultiTask_ interfaces

## PID controller

The class `mtsPID` is part of the [sawControllers](https://github.com/jhu-saw/sawControllers) library.  It provides:
* a PID controller
* clamp requested position within joint limits
* check for PID tracking errors
* configuration using XML, see `sawControllersPID*.xml` files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share)
* _cisstMultiTask_ interfaces

## Arm classes

All the arm classes are part of the [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit) library.  There is a base class (`mtsIntuitiveResearchKitArm`) for:
* powering
* getting data from the PID and IO components
* joint and cartesian motions
* _cisstMultiTask_ interfaces

Since each arm is slightly different, there are three classes derived from the base class:
* `mtsIntuitiveResearchKitPSM`
* `mtsIntuitiveResearchKitECM`
* `mtsIntuitiveResearchKitMTM`

Each of these instantiates some virtual methods to reflect each arm characteristics:
* number of joints and actuators
* arm specific parameters (encoders/potentiometers tolerance, PID tracking error)
* kinematics
* homing procedure including different states (e.g. sterile adapter and tool for PSMs)

ECM and MTM classes currently use a text based configuration file which defines the DH parameters, see `dv*.rob` files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share).  PSMs use a different file format (JSON based) containing the DH parameters as well as actuator/joint coupling matrices, joint limits, ... This file format centralizes all the parameters specific to a given tool (e.g. large needle driver).  See `psm*.json` in the [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share).

Most application should communicate with the arm classes to make sure that state of the system is consistent.  IO and PID components should be used in "read only" mode.

## Tele-operation

The class `mtsTeleOperation` is part of the [sawControllers](https://github.com/jhu-saw/sawControllers) library.  It provides:
* a naive position based tele-operation
* API to set orientation between master and slave
* handles operator present and clutch
* options to lock position or orientation of slave arm
* _cisstMultiTask_ interfaces

## Console

The console class, `mtsIntuitiveResearchKitConsole` is part of the [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit) library.  The main goals of the console are:
* ease of deployment using a lightweight configuration file to describe the components of the system:
  * firewire port
  * periodicity for different components
  * types of arms (MTM, PSM, ECM, derived types...)
  * configurations files for IO, PID, arms with relative paths
  * teleoperation components (optional)
  * setup joints configuration
* maintaining runtime consistency between the different components
  * collect messages from components
  * centralized error handling
  * manage state of arms and controllers
  * handle and dispatch foot pedal events to arms and controllers

The console class also manages all the connections between the _cisstMultiTask_ components which can be tedious when done manually.  The console class is used in `sawIntuitiveResearchKitQtConsoleJSON` and in the ROS application `dvrk_robot/dvrk_console_json`.  Using one of these two applications and a custom console configuration file, one should be able to configure pretty much any system without any C++ programming nor compilation; e.g. just one arm, one pair of arms with or without tele-operation, all arms...

The configuration files are JSON based, see `console*.json` files in sawIntuitiveResearchKit [shared directory](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/master/share).  The subdirectory `jhu-dVRK` contains files for a research kit (i.e. just MTMs and PSMs) while the directory `jhu-daVinci` contains examples for a full system, including setup joints and ECM.

# Qt widgets

All the Qt widgets used for the dVRK are derived from the `mtsComponent` class and either `QObject` or `QWidget`.  This way, they can have both _cisstMultiTask_ interfaces and Qt slots and signals.  There is a couple of things to pay attention to when developing or modifying these widgets:
 * Event handlers are not queued so they must be thread safe.  In most case the event handler should _emit_ a Qt signal
 * Widgets should check if they are hidden or not before performing any computation, specially timer based refreshes

## Robot IOs

The class `mtsRobotIO1394QtWidget` is part of the [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) library.  Since there is usually a single IO component for multiple robots, we use an object factory to create as many widgets as necessary (see class `mtsRobotIO1394QtWidgetFactory`).

The layout will be slightly different based on the robot.
 * Without brakes, PSM and MTM:
  ![IO GUI without brakes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-io.png)
 * With brakes, ECM:
  ![IO GUI with brakes](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-ecm-console.png)
 * Extra widget for buttons (class `prmQtWidgetEventButtonsComponent`, part of [cisstParameterTypesQt](https://github.com/jhu-cisst/cisst) library)
  ![IO GUI buttons](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-buttons.png)

## PID controller

The class `mtsPIDQtWidget` is part of the [sawControllers](https://github.com/jhu-saw/sawControllers) library.

  ![PID GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-pid.png)

## Arm classes

The class `mtsIntuitiveResearchKitArmQtWidget` is part of the [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit) library.

  ![Arm GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-arm.png)

## Tele-operation

The class `mtsTeleOperationQtWidget` is part of the [sawControllers](https://github.com/jhu-saw/sawControllers) library.

  ![Teleop GUI](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-teleop.png)

## Console

The class `mtsIntuitiveResearchKitConsoleQtWidget` is part of the [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit) library.

# ROS

To interface with ROS, we use the `mtsROSBridge` class from the _cisst_ros_bridge_ library (part of the [dvrk_ros](https://github.com/jhu-cisst/cisst-ros) package).  This class allows to interface ROS topics (subscribers and publishers) to _cisstMultiTask_ commands and events.  This library also provides conversion methods between _cisst_ data types and ROS messages.  Finally, we can forward the dVRK messages (status, warning and errors) to the ROS log system.

The main ROS executable is `dvrk_console_json`, part of the _dvrk_ros_/_dvrk_robot_ ROS package.  This simple program uses the `mtsIntuitiveResearchKitConsole` and `dvrk_console` class to configure an application (with Qt Widgets optional) from a JSON console configuration file (see [code](https://github.com/jhu-dvrk/dvrk-ros/blob/master/dvrk_robot/src/dvrk_console_json.cpp)).  The ROS bridge is based on the [cisst CRTK ROS bridge](https://github.com/jhu-cisst/cisst-ros).

## Python

One can use the dVRK ROS topics directly from Python using the ROS Python package.  For simple applications, we also provide a lightweight wrapper with high level methods.  This wrapper hides some of the details of implementation (specially event handling) and convert the (idiotic) ROS messages to usable data types (e.g. KDL frames).   The source code and examples are provided in _dvrk_ros_ in the [`dvrk_python/src`](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_python/src) directory.  The dVRK Python client library is based on the [CRTK Python client](https://github.com/collaborative-robotics/crtk_python_client).

## Matlab

One can use the dVRK ROS topics directly from Matlab using the Robotics System Toolbox for Matlab 2015a or later.   You need to buy this toolbox from MathWorks if you don't already have it.   One nice aspect of this approach is that you can develop your application on any OS supported by Matlab, including Linux, Windows, Mac OS.  For simple applications, we also provide a lightweight wrapper with high level methods.  This wrapper hides some of the details of implementation (specially event handling) and convert the (idiotic) ROS messages to usable data types (e.g. 4x4 homogeneous matrix).   The source code is provided in _dvrk_ros_ in the [`dvrk_matlab`](https://github.com/jhu-dvrk/dvrk-ros/tree/master/dvrk_matlab) directory.  The dVRK Matlab client library is based on the [CRTK Matlab client](https://github.com/collaborative-robotics/crtk_matlab_client).