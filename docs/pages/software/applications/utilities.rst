Utilities
#########

.. _qlacommand:

``qlacommand``
**************

* C++ application with a text based interface using ncurse
* Compatible with **all dVRK controllers**
* https://github.com/jhu-cisst/mechatronics-software/tree/main/tests/

Program that broadcasts messages to all the dVRK controller found on
the FireWire or UDP bus.  Commands include ``reboot``,
``reset-encoder-preload``, ``close-relays``, ``open-relays``...


.. _io-config-generator:

``dvrk-io-config-generator.py``
*******************************

* Python script with text interface (not ROS)
* Compatible with **all dVRK active arms**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/io-config-generator

Usage is described in the :ref:`configuration generators
<io-config-generator-use>` section.

.. _remove-logs:

``dvrk-remove-logs.py``
***********************

* Python script with text interface (not ROS)
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/config-generator

Script used to locate and delete log and backup files. Everytime a
cisst/SAW based application (like the dVRK main system program) is
started, a log file is created (named ``cisstLog-xxx.txt``). After a
while, you will find that your home directory is cluttered with these
log files and it would be nice to remove them. The script
``dvrk-remove-logs.py`` can be used to locate all the log files in the
current directory (and sub-directories) and delete them.  This script
also looks for backup files created by some of the dVRK calibration
applications. By default, the script displays the list of files found
and prompts the user to confirm deletion.  With the ``-f`` (force)
option, files are automatically deleted without any prompt.  For
example:

.. code-block:: bash

   # remove every log/backup file in your user directory
   cd ~/
   dvrk-remove-logs.py -f


.. _dvrk_reset_teleoperation:

``dvrk_reset_teleoperation.py``
*******************************

* ROS Python script with a text based interface, it depends on
  CRTK/dVRK Python to communicate with the dVRK system.
* ROS package ``dvrk_python``
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

This scripts will stop the teleoperation components (see :ref:`dVRK
system <system>`), move the MTMs at their zero position, move the
PSM's shaft rotation to zero and restart the teleoperation.  The goal
of the script is to "reset" the kinematic for both the MTMs and PSMs.
This is useful when the arms get into odd poses after a bit of
teleoperation.::

   usage: dvrk_reset_teleoperation.py [-h] -m {MTML,MTMR} [{MTML,MTMR} ...] [-p {PSM1,PSM2,PSM3} [{PSM1,PSM2,PSM3} ...]]


.. _hrsv_widget:

``hrsv_widget``
********************

* C++ ROS application with Qt widgets, doesn't depend on cisst/SAW
* ROS package ``dvrk_hrsv_widget``
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/ros/dvrk_hrsv_widget

This application creates two small widgets which display the current
state of the dVRK system. It will show if the operator is present, if
the clutch or camera pedal is pressed as well as which PSM is
teleoperated by which MTM.  ROS is used to track the system's state.
The two widgets can be dragged on top of the left and right views in
the stereo display. This is a poor man's version of the UI in a
clinical system but it can still help users who are not familiar with
the system.

.. _sd-card-updater:

``dvrk-sd-card-updater.py``
***************************

* Python script with text interface (not ROS)
* Compatible with **dVRK DQLA-based Classic controllers and all Si controllers**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/sd-card-updater

Script used to download, uncompress and copy the latest firmwares for
the DQLA and dRAC based controllers as well as the ESPM programmers.
The scripts waits for an SD card to be inserted in the PC.  It will
then mount it, copy the files and unmount so the user can safely
remove the card.  The user can insert all their SD cards one after
another without restarting the script.

.. _dvrk_teleoperation:

``dvrk_teleoperation.py``
*************************

* ROS Python script
* ROS package ``dvrk_python``
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

Script for Python/ROS-based teleoperation, useful as an example/template of how the dVRK teleoperation can be customized. See also :ref:`derived components <components-derived>` for an example of customizing the C++ teleoperation component directly. The script is given ROS namespaces for an MTM and PSM to begin teleoperation with, which could be an actual dVRK MTM and PSM, or the MTM could instead be a haptic device such as a `ForceDimension <https://github.com/jhu-saw/sawForceDimensionSDK>`_. When running, teleoperation works very similarly to the built-in dVRK teleoperation.

To run teleoperation, first start a :doc:`dVRK system <system>` for the arms you want to teleoperate. This should be a system that *doesn't* include a built-in teleoperation componenent, such as::

   ros2 run dvrk_robot dvrk_system -j system-MTML-PSM2.json

Next, start the teleoperation script with your chosen arms::

   ros2 run dvrk_python dvrk_teleoperation.py -m <MTM> -p <PSM>

If you don't have dVRK foot pedals, you can use the ``-c <cluch topic>`` and ``-o <operator pedal>`` topics to use a different source (these topics should each provide ``sensors_msgs/msg/Joy`` with one button). Alternatively, you can set ``-o`` to disable the operator pedal, note however that in this mode teleoperation will begin as soon as any MTM activity is detected so please be careful.

If you are using a haptic device (e.g. ForceDimension/Falcon) as your MTM which has an unactuated wrist, the teleoperation script will not be able to align the MTM to match the PSM orientation. In this case, you must use the ``-n`` option to skip alignment.
