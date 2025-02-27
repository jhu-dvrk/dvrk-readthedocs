.. _calibration-classic-pots:

Potentiometers
**************

Introduction
============

The potentiometers on the dVRK are used for:

* Homing, i.e. they provide an absolute reference to define the zero
  position
* Safety, i.e. by reading both encoders and potentiometers
  continuously one can detect discrepancies

The potentiometer values are read as voltages and converted to SI
positions (radians for revolute joints and meters for prismatic
joints). The conversion is a linear function based on an offset and a
scale, i.e. ``position = offset + scale * voltage``.  Intuitive
Surgical performed an initial calibration for all arms and can provide
these values in a ``.cal`` file.  Using these ``.cal`` file and the
dVRK config generator, we get the IO XML
``sawRobotIO1394-<arm>-<serial>.xml`` files used for the dVRK.  See
:ref:`configuration generators<config-generators>`.

The problem is that these values are partially based on the
electronics used during the calibration.  As such, they are a bit off.
We developed two different strategies to calibrate the scales and
offsets.

For the scales, the simplest solution is to rely on the encoders.  We
generate a large motion on each actuator and collect both the encoder
and potentiometer values.

For the offsets, it is a bit more challenging since we need to
identify a zero position based on mechanical properties.

* The zero position can be visualized using the dVRK in kinematic
  simulation mode with RViz.  To do so, launch ``roslaunch dvrk_model
  arm.launch generation:=Classic arm:=ECM`` for the ECM zero.  You
  can replace ``ECM`` by ``PSM1``, ``MTML`` or ``MTMR`` to visualize
  the zero position of different arms.

  .. figure:: /images/Classic/PSM/psm-zero-rviz.png
     :width: 300
     :align: center

     Classic PSM "home" or "zero" position

  .. figure:: /images/Classic/ECM/ecm-zero-rviz.png
     :width: 300
     :align: center

     Classic ECM "home" or "zero" position

  .. figure:: /images/Classic/MTM/mtms-zero-rviz.png
     :width: 300
     :align: center

     Classic MTML and MTMR "home" or "zero" position

* For the calibration process, you need to hold the arm in zero
  position.  You can do this with the arm powered or not.  When
  powered, you can use the GUI arm widget to control the joints
  position and get the arm closed to the zero position (introduced in
  dVRK 2.0).

* This page contains a simple solution to calibrate the potentiometer
  offsets for the PSM last 4 actuators.  We also provide a method to
  `calibrate the PSM 3rd potentiometer
  <calibration-classic-pots-depth>`.

.. warning::

   It is important to calibrate the potentiometer scales before the
   offsets!

Requirements
============

For the ECM, make sure the brakes are properly calibrated.  This
requires to calibrate both the controller current (see above) and the
power to release the brakes.

As for the other calibration steps, you need to have all the
configuration files generated, the C++ code compiled and the current
calibration performed.  Furthermore, the current implementation
requires the ROS bridges and Python.  Make sure you compiled your dVRK
software stack using ``catkin`` or ``colcon`` (see
:ref:`compilation<compilation>`).

For the offsets, we need a physical mechanism to maintain the arm in
zero position (or any known position).  We currently have a fairly
easy solution for the last 4 joints of the PSM.  The four metal
bars/gears are in zero positions when aligned.  We tried different
methods and got similar results, so you should use whatever is the most
convenient for you:

 * Calibration template made of plexiglass plate with holes for the
   pins on the 4 wheels.

   .. figure:: /images/Classic/PSM/psm-pot-calib-plate-in-place.jpg
      :width: 300
      :align: center

      PSM calibration plate for the last 4 actuators

 * Two vertical bars pushing on the sides using Lego pieces.  One can
   probably use a rubber band to pull the two vertical bars against
   the gears.

   .. figure:: /images/Classic/PSM/psm-pot-calib-lego-in-place.jpg
      :width: 300
      :align: center

      PSM calibration using Legos for the last 4 actuators

CAD/STL/DWG files for templates to hold the last 4 joints can be found
in https://github.com/jhu-dvrk/dvrk-calibration-parts

.. _calibration-classic-pots-scale:

Calibrating scales
==================

These instructions are for all arms, PSMs, MTMs and ECM.  For the
calibration, one needs to start the ``dvrk_console_json`` application
for the arm to be calibrated (see :ref:`dVRK console <console>`).
Since we also need the low level data (potentiometer values), we have
to provide the ``-K`` option.  For example, to calibrate a PSM2,
command line options for ``dvrk_console_json`` should look like:

.. code-block:: bash

   # In directory <my-config-dir>
   # directory with your sawRobotIO1394-PSM2-00000.xml configuration files
   rosrun dvrk_robot dvrk_console_json -j <my-config-dir>/console-PSM2.json -K -C

.. note::

   The ``-C`` command line otion (added in release 2.0.1) allows to
   run the dVRK console without the potentiometer safety checks
   (**C**\ alibration mode).  Otherwise, with very poorly calibrated
   potentiometer parameters, the application would keep shutting down,
   preventing users to calibrate their potentiometer parameters.  With
   ``-C``, the console application also resets the encoder preloads on
   exit.  This is to avoid using bad encoder preloaded values (based
   on poor potentiometer values) on the next run.

The file ``console-PSM2.json`` is specific to each system since it
points to your ``sawRobotIO1394-PSM2-00000.xml`` file.

In a separate shell, start the :ref:`potentiometer calibration script
<dvrk_calibrate_potentiometers>` using the following command line:

.. code-block:: bash

   # In directory <my-config-dir>
   rosrun dvrk_python dvrk_calibrate_potentiometers.py -t scales -a PSM2 -c sawRobotIO1394-PSM2-00000.xml

Make sure you use the same ``sawRobotIO1394-XXX-00000.xml`` for the
calibration script and the console application! The file name can be
found in the console-PSM2.json file you're using.

The calibration script will query the arm serial number from the XML
file and will display it.  The console application will do the same
and display the serial number in the IO Qt widget.  This ensures that
both applications are using an XML file specific to the arm you are
trying to calibrate.  But, if you happen to use different copies of
the configuration file for your arm, the current system has no way to
detect it.  So, make sure you are using the same file for both
applications (console and calibration script).

You will have to acknowledge a few prompt messages, including a
warning regarding large motions during the calibration.  The following
two videos can give you a sense of the space required around the arm:

* MTM: https://youtu.be/tixIjsO6BT0
* PSM (the ECM performs a similar motion): https://youtu.be/Pl6NQTwF9nU

::

   Calibrating scales using encoders as reference
   Values will be saved in:  pot_calib_scales_sawRobotIO1394-PSM2-00000.csv
   To start with some initial values, you first need to "home" the robot.  When homed, press [enter]
   Since you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]
   The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed

.. caution::

   For the scale calibration, we try to use a wide range of positions, so the arm will pretty much go from joint limits to joint limits.  Make sure there are no obstacles in the way!

The result should look like:

::

 index | old scale  | new scale  | correction
  0    | -44.329108 | -43.493731 |  1.019207
  1    | -29.309363 | -28.708860 |  1.020917
  2    |  60.074692 |  59.488202 |  1.009859
  3    | -78.384293 | -78.608156 |  0.997152
  4    | -77.862774 | -78.044577 |  0.997671
  5    | -78.279990 | -78.374442 |  0.998795
  6    | -79.427331 | -79.140566 |  1.003623

In this case you can see corrections as high as 2% on the third joint
(index 2).  Press `y[enter]` to save the results in a new XML file.
You can review the changes with `meld` or your preferred diff tool.
If the changes make sense, replace your default XML configuration file
with the new one:

Then stop the dVRK console application and restart it with the updated
XML file to re-run the calibration script.  The results should
improve:

::

 index | old scale  | new scale  | correction
  0    | -43.493731 | -43.490507 |  1.000074
  1    | -28.708860 | -28.694983 |  1.000484
  2    |  59.488202 |  59.479411 |  1.000148
  3    | -78.608156 | -78.605950 |  1.000028
  4    | -78.044577 | -78.041157 |  1.000044
  5    | -78.374442 | -78.373988 |  1.000006
  6    | -79.140566 | -79.138265 |  1.000029

There is usually no point to save the results of the second pass.

.. _calibration-classic-pots-offset:

Calibrating offsets
===================

These instructions are for all arms, but we only know how to properly
hold the joints at their zero position for the last 4 joints of the
**PSMs**.  If you need to calibrate offsets on different arms (MTM,
ECM), you will need to figure out a way to constrain the arm to its
zero position (mechanical zero).

For the scales' calibration, you first need to start the console
application and power the arm.  If the arm can power with the existing
potentiometer offsets, home the arm.  You can then either keep the arm
powered and use the motors to position it close to its mechanical
zero.  For the ECM and PSM, when the arm is maintained in position
using its motors, you can use the "clutch" button to release the PID
controller and position the arm manually.  For all arms, you can also
use the ROS topics to send `move` goals or use the Qt GUI (dVRK 2.0
and above).  Once the arm is close to its mechanical zero position,
you can use the script below.

In a separate shell, start the calibration script using the following
command line:

.. code-block:: bash

   # In directory <my-config-dir>
   rosrun dvrk_python dvrk_calibrate_potentiometers.py -t offsets -a PSM2 -c sawRobotIO1394-PSM2-00000.xml

Follow the instructions and place the calibration template (either
Lego bars or plexiglass plate) when prompted to.  The result should
look like:

::

 index | old offset  | new offset  | correction
  0    |   99.441352 |   99.441352 |  0.000000
  1    |   68.032665 |   68.032665 |  0.000000
  2    |  -14.153006 |  -14.153006 |  0.000000
  3    |  176.339392 |  177.817309 | -1.477917
  4    |  176.606849 |  176.959943 | -0.353094
  5    |  174.920864 |  175.741625 | -0.820761
  6    |  179.924389 |  179.851204 |  0.073185

For the MTMs or ECM, the script will save all joint offsets.  For the
PSMs, since we know there is an easy way to calibrate the last 4 joint
offsets, the script will prompt you to figure out if you should save
all the joints or only the last 4.  If you are using the Lego bars or
template describe above, **DO NOT** save all, just save the last 4.

Then stop the console application, make sure you restart it with the
updated XML file and re-run the calibration script.  The results
should improve:

::

 index | old offset  | new offset  | correction
  0    |   99.441352 |   99.441352 |  0.000000
  1    |   68.032665 |   68.032665 |  0.000000
  2    |  -14.153006 |  -14.153006 |  0.000000
  3    |  177.817309 |  177.817577 | -0.000269
  4    |  176.959943 |  176.986576 | -0.026634
  5    |  175.741625 |  175.801207 | -0.059582
  6    |  179.851204 |  179.858797 | -0.007594

Similar to the scales, there is usually no point to save the results
of the second pass for the offsets.
