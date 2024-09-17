Si
##

PSMs and ECM
************

Introduction
============

The Si arms on the patient side (PSM and ECM) use relative encoders
and digital potentiometers.  The digital potentiometers are absolute
position sensors.  They can be used to preload the encoders so these
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


The program will try to power the arm and release the brakes.  At that point, you need to move each and every joint from one limit to the other.  To release the brakes, press the clutch button (white on top of arm).

Videos for:

* PSM: https://youtu.be/det51NzsHvA
* ECM: https://youtu.be/rtlOz3qVc34

.. warning::
   There is no gravity compensation so make sure you keep one hand on the endoscope holder.

As you move the arm around you should see the counters go up for each
joint.  Once you've moved all the joints from limit to limit, all the
counters should be close to 3000.  You can then hit any key to
proceed.  If everything went well, a new JSON file will be created with
the generated lookup table.

If the measured range of motion doesn't match the expected range, you
will get an error message.  You will have to repeat the procedure
making sure you hit all the joint limits.


SUJs
****
