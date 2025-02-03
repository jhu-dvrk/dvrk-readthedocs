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

``dvrk-config-generator.py``
*****************************

* Python script with text interface (not ROS)
* Compatible with **all dVRK active arms**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/config-generator

Usage is described in the :ref:`configuration generators
<io-config-generator-use>` section.

.. _remove-logs:

``dvrk-remove-logs.py``
***********************

* Python script with text interface (not ROS)
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/config-generator

Script used to locate and delete log and backup files. Everytime a
cisst/SAW based application (like the dVRK main console program) is
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
  CRTK/dVRK Python to communicate with the dVRK console.
  ...``
* ROS package ``dvrk_python``
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

This scripts will stop the teleoperation components (see :ref:`dVRK
console <console>`), move the MTMs at their zero position, move the
PSM's shaft rotation to zero and restart the teleoperation.  The goal
of the script is to "reset" the kinematic for both the MTMs and PSMs.
This is useful when the arms get into odd poses after a bit of
teleoperation.

::

   usage: dvrk_reset_teleoperation.py [-h] -m {MTML,MTMR} [{MTML,MTMR} ...] [-p {PSM1,PSM2,PSM3} [{PSM1,PSM2,PSM3} ...]]


.. _hrsv_widget:

``hrsv_widget``
********************

* C+ ROS application with Qt widgets, doesn't depend on cisst/SAW
* ROS package ``dvrk_hrsv_widget``
* Compatible with **all dVRK setups**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/ros/dvrk_hrsv_widget

This application creates two small widgets which display the current
state of the dVRK console. It will show if the operator is present, if
the clutch or camera pedal is pressed as well as which PSM is
teleoperated by which MTM.  ROS is used to track the console's state.
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
