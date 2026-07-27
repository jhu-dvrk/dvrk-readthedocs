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
   dvrk-io-config-generator.py -a ECM -g Si -H dRA1 -s 123456  # replace 123456 by your serial number and ECM by PSM1, PSM2 or PSM3 if needed

To calibrate the potentiometers, use:

.. code-block:: bash

   sawIntuitiveResearchKitSPotentiometersCalibration -c sawRobotIO1394-ECM-123456.json  # use the json file for your arm!

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

The Si SUJ calibration computes the scale and offset used to convert
the primary and secondary analog potentiometer voltages into SI joint
positions.  The script records the minimum and maximum voltage seen
while each setup joint is moved through its mechanical range, then
maps that voltage range to the expected Si SUJ joint limits.

To be able to move the SUJ freely, you will need to be able to release
the brakes. For each SUJ arm (e.g. ECM SUJ) you plan to calibrate, you
need to power on the dVRK controller for the corresponding active arm
(e.g. ECM) and turn on the ``dvrk_system``.  If you want to
calibrate all the SUJ arms at the same time, you will need a system
configuration file with all the active arms.

The system configuration must include the SUJ Si and the main ROS
node (``dvrk_robot/dvrk_system``) must be launched with the
extra command line option ``-s`` (or ``--suj-voltages``).  The SUJ Si
configuration also uses one IO calibration file per mounted arm, named
``sawRobotIO1394-SUJ-Si-<arm>-<serial>.json``.  These files are
created by the IO configuration generator when generating Si PSM or
ECM configuration files.  Run the calibration script from the
directory containing these files so it can find and update them.

For example, in a ROS 2 workspace:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   cd <my-config-dir>
   ros2 run dvrk_robot dvrk_system -j system-SUJ-ECM-PSM1-PSM2-PSM3.json -s

Once the dVRK system is started, you should be able to see the
potentiometer voltages using ROS topics.  For example:

::

   ros2 topic echo /SUJ/PSM1/primary_voltage/measured_js
   ros2 topic echo /SUJ/ECM/secondary_voltage/measured_js

The :ref:`calibration script <dvrk_calibrate_suj>` is
``dvrk_calibrate_suj.py``.  It can be found in the ROS package
``dvrk_python``.

* ROS 1: ``source ~/catkin_ws/devel/setup.bash`` and ``rosrun dvrk_python dvrk_calibrate_suj.py``
* ROS 2: ``source ~/ros2_ws/install/setup.bash`` and ``ros2 run dvrk_python dvrk_calibrate_suj.py``

The script opens a Qt window with one row for each primary and
secondary potentiometer set.  Each joint column shows the minimum and
maximum voltage observed so far and the current measured range.  The
``Save ECM``, ``Save PSM1``, ``Save PSM2``, and ``Save PSM3`` buttons
are enabled only when the matching
``sawRobotIO1394-SUJ-Si-<arm>-<serial>.json`` file is present in the
current directory.

Calibrate one SUJ arm at a time:

#. Release the brakes for the arm.
#. Move each setup joint from one mechanical limit to the other.
#. Stay at each limit for about one second so the voltage readings can
   settle.
#. Confirm in the GUI that the displayed range increased for every
   primary and secondary potentiometer on that arm.
#. Press the matching ``Save <arm>`` button.

When saving, the script computes new ``primary_measured_js`` and
``secondary_measured_js`` scale/offset entries, renames the old JSON
file with a timestamped ``-backup-`` suffix, and writes the updated
calibration to the original filename.  Restart ``dvrk_system`` after
saving so the new calibration is loaded.

.. caution::

   The PSM3 SUJ has a potentiometer dead-zone on its fifth joint.  Past
   a certain point, the values reported in the GUI are meaningless.
   Identify the dead-zone and avoid it during calibration.  The GUI
   provides ``Reset PSM3 joint 5`` to reset the observed range for that
   joint after returning to a valid position.  The script assumes the user will move the joint between the mechanical limit and the point where the etch marks on the joint are aligned.
