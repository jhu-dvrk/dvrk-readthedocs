Debugging
#########

.. _qladisp:

``qladisp``
***********

* C++ application with a text based interface using ncurse
* Compatible with all dVRK controllers
* https://github.com/jhu-cisst/mechatronics-software/tree/main/tests/

.. note::

   It can be compiled without cisst/SAW, ROS and the full dVRK stack using
   CMake instead of catkin or colcon.

qladisp documentation goes here

.. _sawrobotioqtconsole:

``sawRobotIOQtConsole``
***********************

* C+ application with a Qt based GUI, it depends on cisst/SAW
* Compatible with all dVRK controllers
* https://github.com/jhu-saw/sawRobotIO1394/tree/main/core/applications/Qt

Used for Classic ECM and Si ECM/PSMs :ref:`current brake calibration
<calibration-classic-ecm>` with *direct control*.

Also ``robot_io robot_io_console`` for ROS topics.  Can be used to
create your own application directly on top of IO component.

.. _sawintuitiveresearchkitqtpid:

``sawIntuitiveResearchKitPID``
******************************

* C+ application with a Qt based GUI, it depends on cisst/SAW
* Compatible with all dVRK active arm controllers (not SUJ)
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications

Can be used to tune PID without any other component running.  No
built-in homing procedure sos user have to use *direct control* in IO
widget to preload the encoders if they want absolute positions.
