.. _applications:

************
Applications
************

`dvrk_console_json`
###################

Run
***

rosrun dvrk_robot dvrk_console_json

sawIntuitiveResearchKitQtConsoleJSON

Command line options
********************

Configuration
*************

Using component manager configuration files

Calibration
###########

`sawRobotIO1394CurrentCalibration`
**********************************

`dvrk_calibrate_potentiometers.py`
**********************************

`dvrk_calibrate_potentiometer_psm.py`
*************************************

`sawIntuitiveResearchKitGripperCalibration`
*******************************************

`sawIntuitiveResearchKitSPotentiometersCalibration`
***************************************************

Debugging
#########

.. _qladisp:

`qladisp`
*********

qladisp documentation goes here

`sawRobotIOQtConsole`
*********************

Also `robot_io robot_io_console` for ROS topics.  Can be used to
create your own application directly on top of IO component.

`sawIntuitiveResearchKitQtPID`
******************************

Can be used to tune PID without any other component running.  No
built-in homing procedure so user have to use direct control in IO
widget to preload the encoders if they want absolute positions.

Utilities
#########

`qlacommand`
************

`dvrk-remove-logs.py`
*********************

`dvrk_hrsv_widget`
******************

`dvrk-sd-card-updater.py`
*************************
