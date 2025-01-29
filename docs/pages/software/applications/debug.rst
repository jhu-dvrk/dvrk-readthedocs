Debugging
#########

.. _qladisp:

``qladisp``
***********

qladisp documentation goes here

.. _sawrobotioqtconsole:

``sawRobotIOQtConsole``
***********************

Used for Classic ECM and Si ECM/PSMs :ref:`current brake calibration
<calibration-classic-ecm>`.

Also ``robot_io robot_io_console`` for ROS topics.  Can be used to
create your own application directly on top of IO component.

.. _sawintuitiveresearchkitqtpid:

``sawIntuitiveResearchKitQtPID``
********************************

Can be used to tune PID without any other component running.  No
built-in homing procedure so user have to use direct control in IO
widget to preload the encoders if they want absolute positions.
