.. _io:

Low level IOs
#############

Functionalities
***************

The class ``mtsRobot1394`` is part of the `sawRobotIO1394
<https://github.com/jhu-saw/sawRobotIO1394>`_ library.  It is built on
top of the *Amp1394* library used to communicate with the dVRK
controllers (part of :ref:`mechatronic software <amp1394>`).

It provides:

* Conversion to/fromfrom integers from/to SI units
* Support for motors and brakes (e.g. Classic ECM and Si PSMs/ECM)
* Software based velocity estimation based on encoder counts and time
  since last change
* Extra safety checks (consistency between potentiometers and
  encoders, compare required and measured current)

The classes ``mtsDigitalInput1394`` and ``mtsDigitalOutput1394`` are
used for all digital inputs and outputs.

Configuration files
*******************

Configuration files use XML or JSON (future versions)

Applications
************

  * *sawIntuitiveResearchKitQtConsoleJSON* and ROS ``dvrk_robot/dvrk_console_json``
  * *sawRobotIO1394QtConsole* and ``ROS robot_io/robot_io_console``

Widgets
*******

For any arm without brakes (Classic MTMs and PSMs), the IO widget
display all the information regarding the state of the controller and
axes.  By default, this widget only has two active buttons, *Direct
control* and *Plot position*.

.. figure:: /images/gui/gui-Classic-MTM-io.png
   :align: center

   IO widget for an arm without brakes (MTM)

When *Direct control* is enabled, the user can directly send commands
to turn power on/off, close the safety relays, change the data
communication watchdog (i.e. FireWire or Ethernet), reset all the
encoders to zero or preload the encoders based on the potentiometer
values.  Furthermore, the user can use the sliders to directly control
the current sent to any motor or brake.

.. caution::

   None of the operations allowed in *Direct control* should be
   performed if there is a PID or arm component running (e.g. with
   *sawIntuitiveResearchKitQtConsoleJSON*)

.. caution::

   When using *Direct control*, please keep the emergency stop handy
   so you can easily power off everything!

.. figure:: /images/gui/gui-Classic-ECM-io.png
   :align: center

   IO widget for an arm without brakes (MTM)

.. figure:: /images/gui/gui-Classic-MTML-gripper-io.png
   :align: center

   IO widget for a single analog input (MTM gripper)

.. figure:: /images/gui/gui-Classic-MTML-io-plot.png
   :align: center

   IO widget plotting function to compare encoders and potentiometers
