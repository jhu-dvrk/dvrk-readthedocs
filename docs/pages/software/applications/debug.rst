Debugging
#########

.. _qladisp:

``qladisp``
***********

* C++ application with a text-based interface using ncurses
* Compatible with all dVRK controllers
* https://github.com/jhu-cisst/mechatronics-software/tree/main/tests/

.. note::

   It can be compiled without cisst/SAW, ROS and the full dVRK stack using
   CMake instead of catkin or colcon.

Displays the status and sensor feedback of all connected FPGA1394/QLA/DQLA/dRAC boards found on the FireWire or Ethernet bus (see :ref:`checking connection to controllers <usage-real>`).

.. _sawrobotioqtconsole:

``sawRobotIOQtConsole``
***********************

* C++ application with a Qt-based GUI, it depends on cisst/SAW
* Compatible with all dVRK controllers
* https://github.com/jhu-saw/sawRobotIO1394/tree/main/core/applications/Qt

Used for Classic ECM and Si ECM/PSMs :ref:`current brake calibration
<calibration-classic-ecm>` with *direct control*.

Also ``robot_io robot_io_console`` for ROS topics.  Can be used to
create your own application directly on top of IO component.

.. _sawintuitiveresearchkitqtpid:

``sawIntuitiveResearchKitPID``
******************************

* C++ application with a Qt-based GUI, it depends on cisst/SAW
* Compatible with all dVRK active arm controllers (not SUJ)
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications

Can be used to tune PID without any other component running.  No
built-in homing procedure, so users have to use *direct control* in the IO
widget to preload the encoders if they want absolute positions.
