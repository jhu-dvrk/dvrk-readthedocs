.. _calibration_si:

.. include:: /includes/logic-view-Si-io.rst

**
Si
**

.. warning::

   This section is for the Si arms only!


.. _calibration-si-psm-ecm-pots:

PSMs and ECM potentiometers
***************************

Introduction
============

The Si arms on the patient side (PSM and ECM) use relative encoders
and digital potentiometers.  The digital potentiometers are absolute
position sensors.  They can be used to preload the encoders, so these
can report absolute positions.  The dVRK implementation relies on the
fact that there is a limited number of possible values for the digital
potentiometers (4096) so we can create a lookup table to associate an
absolute position in radians or meters to each potentiometer value
(aka index).  To find the values in the lookup table, we're using the
fact that we know:

* The encoder resolution
* The expected range of motion for each joints

Therefore, by moving each joint from one limit to another we can find
the measured range for a given joint and, recenter the measured range
to the expected range.

Procedure
=========

The first step is to source your ROS workspace:

* ROS 1: ``source ~/catkin_ws/devel/setup.bash``
* ROS 2: ``source ~/ros2_ws/install/setup.bash``

.. warning::
   It is **extremely important** to make sure you always match the serial number from the arm you're using!

The configuration files should have been created using something like:

.. code-block:: bash

   cd <where_ever_you_want_your_files>
   dvrk-config-generator.py -a ECM -g Si -H dRA1 -s 123456  # replace 123456 by your serial number and ECM by PSM1, PSM2 or PSM3 if needed

To calibrate the potentiometers, use:

.. code-block:: bash

   sawIntuitiveResearchKitSPotentiometersCalibration -c sawRobotIO1394-ECM-123456.xml  # use the xml file for your arm!

The :ref:`calibration program
<sawintuitiveresearchkitsipotentiometerscalibration>` will try to
power the arm and release the brakes.  At that point, you need to move
each and every joint from one limit to the other.  It's important to
get values as close as possible to the mechanical limits so hit each
limit a couple of times (not too brutally, see videos below).  To
release the brakes, press the clutch button (white on top of arm).

.. warning::
   There is no gravity compensation so make sure you keep one hand on the endoscope holder.

Videos for:

* PSM: https://youtu.be/det51NzsHvA
* ECM: https://youtu.be/rtlOz3qVc34

As you move the arm around you should see the counters go up for each
joint.  Once you've moved all the joints from limit to limit, all the
counters should be close to 3000. You can then hit any key to
proceed. If everything went well, a new JSON file will be created with
the generated lookup table.

If the measured range of motion doesn't match the expected range, you
will get an error message. You will have to repeat the procedure
making sure you hit all the joint limits.


.. _calibration-si-suj-pots:

SUJ potentiometers
******************

The SUJ Si are fairly new for the dVRK and the calibration process is
not thoroughly tested. For now, we use the mechanical limits to
identify the conversion factors from the analog potentiometers to SI
units. We don't use the "home" grooves on the links except for the
PSM3 SUJ. There is a joint that moves past the analog potentiometers
and reports useless values for a certain range of motion.

todo Anton needs to document this.

To be able to move the SUJ freely, you will need to be able to release
the brakes. For each SUJ arm (e.g. ECM SUJ) you plan to calibrate, you
need to power on the dVRK controller for the corresponding active arm
(e.g. ECM) and turn on the ``dvrk_console_json``.  If you want to
calibrate all the SUJ arms at the same time, you will need a console
configuration file with all the active arms.

The console configuration must include the SUJ Si and the main ROS
node (``dvrk_robot/dvrk_console_json``) must be launched with the
extra command line option ``-s`` (or ``--suj-voltages``).  Once the
dVRK console is started, you should be able to see the potentiometer
voltages using ROS topics.  For example:

::

   ros2 topic echo /SUJ/PSM1/primary_voltages/measured_js
   ros2 topic echo /SUJ/ECM/secondary_voltages/measured_js

The :ref:`calibration script <dvrk_calibrate_suj>` is
``dvrk_calibrate_suj.py``.  It can be found in the ROS package
``dvrk_python``.

* ROS 1: ``source ~/catkin_ws/devel/setup.bash`` and ``rosrun dvrk_python dvrk_calibrate_suj.py``
* ROS 2: ``source ~/ros2_ws/install/setup.bash`` and ``ros2 run dvrk_python dvrk_calibrate_suj.py``

Since SUJ calibration relies on the joint limits, i.e. you will have
to move each joint from its minimum to maximum limit.  The
potentiometer readings are fairly slow so make sure you stay at the
position limit for a second or so.  As you move the SUJs around, each
potentiometer range displayed in the GUI should increase.  Start with
one SUJ, move each joint from one mechanical limit to the other.

You can at any point see the results of the calibration by hitting the
**Show** button.  When you hit **Show**, the result of the calibration
are displayed in the terminal, in JSON format.  You can then select
the lines corresponding to the SUJ arm you just calibrated and
copy/paste them in your SUJ Si JSON configuration files (replace the
existing lines in the file).

.. caution::

   There is a potentiometer dead-zone on the third joint of the SUJ
   PSM3.  Past a certain point, the values reported in the GUI are
   meaningless.  For this specific joint, you have to identify the
   dead-zone and avoid it during the calibration.  The range of motion
   to calibrate this joint should be from the lower mechanical limit
   to the point where the hash marks (grooves) line up.
