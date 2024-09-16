.. _Compilation:

***********
Compilation
***********

ROS1
====

We suggest 2 different ways to retrieve all the dVRK required source
repositories and compile them.  These methods are for Linux/ROS users
only and rely on the catkin build tools.

`catkin build` and `rosinstall`
-------------------------------

The `rosinstall` configuration file is provided in dVRK 2.x and higher
but can also be used with older versions (see `dvrk-ros
<https://github.com/jhu-dvrk/dvrk-ros)>`_.  The `rosinstall` file
defines all the github repositories that need to be cloned in your
workspace as well as which branches.

Debian packages
^^^^^^^^^^^^^^^

This section assumes you already have ROS installed (see `www.ros.org
<https://www.ros.org>`_).  You will need to install a few more
packages for the dVRK software:

.. tabs::

   .. tab:: Ubuntu 16.04

      Ubuntu 16.04 with ROS Kinetic:

      .. code-block:: bash

         sudo apt-get install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig flite sox espeak cmake-curses-gui cmake-qt-gui libopencv-dev git subversion gfortran libcppunit-dev qt5-default python-wstool python-catkin-tools

   .. tab:: Ubuntu 18.04

      Ubuntu 18.04 with ROS Melodic:

      .. code-block:: bash

         sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev  libbluetooth-dev python-wstool python-catkin-tools

   .. tab:: Ubuntu 20.04

      Ubuntu 20.04 with ROS Noetic

      .. code-block:: bash

         sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev python3-pyudev python3-wstool python3-catkin-tools python3-osrf-pycommon``

.. warning::
   For any dVRK software version greater than 2.1:
   
   * Ubuntu 16.04 support has been dropped
     
   * Ubuntu 18.04 support requires clang instead of gcc.  You will need to install clang with ``sudo apt install clang`` and configure your workspace using: ``catkin config --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++``

Catkin workspace, clone and build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're using ROS Noetic and the master branch, you can just
copy/paste the following block of commands in a terminal.  For other
configurations, make sure your replace ``noetic`` by ``melodic`` or
whatever version of ROS you're using.  For the ``devel`` branches,
replace ``master`` by ``devel``.

.. code-block:: bash

   source /opt/ros/noetic/setup.bash # or use whatever version of ROS is installed!
   mkdir ~/catkin_ws                  # create the catkin workspace
   cd ~/catkin_ws                     # go in the workspace
   wstool init src                    # we're going to use wstool to pull all the code from github
   catkin init                        # create files for catkin build tool
   catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
   cd src                             # go in source directory to pull code
   wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/master/dvrk_ros.rosinstall # or replace master by devel
   wstool up                          # now wstool knows which repositories to pull, let's get the code
   catkin build --summary             # ... and finally compile everything

Environment variables
^^^^^^^^^^^^^^^^^^^^^

This is recommended if you're going to use a single catkin workspace
(as most users do):
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#setting-up-your-environment-variables-for-ros

`catkin build` and git submodules
---------------------------------

This approach is a bit more complicated and will add some extra
repositories for SAW components not needed for the dVRK software.
Compilation time will be slightly longer.  Most repositories in
cisst/SAW will be cloned using git sub-modules.

cisstNetlib, cisst, SAW components and cisst-ros bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will first need to build cisst and its dependencies.  Follow the
instructions provided for [cisst/SAW catkin
build](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros)
and then come back to this page for the dVRK/ROS specific packages.

dvrk-ros
^^^^^^^^

These packages are not part of the cisst-saw repositories so you have
to clone them manually.  You first need to download the cisst-saw
libraries and components (see instructions above) and then do:

.. code-block:: bash

   cd ~/catkin_ws/src
   git clone https://github.com/jhu-dvrk/dvrk-ros
   git clone https://github.com/jhu-dvrk/dvrk-gravity-compensation
   git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
   git clone https://github.com/collaborative-robotics/crtk_python_client crtk/crtk_python_client
   git clone https://github.com/collaborative-robotics/crtk_matlab_client crtk/crtk_matlab_client
   catkin build --summary

ROS 2
=====

This code hasn't been tested extensively.  We welcome any feedback.
The following has been tested on Ubuntu 20.04 with ROS 2 Galactic and
Ubuntu 22.04 with ROS 2 Humble.

ROS 2 and extra packages
------------------------

Install ROS 2 following instructions from `www.ros.org
<https://www.ros.org>`_.  The following packages might not be
installed along the ROS desktop but are needed for all ROS
distributions:

.. code-block:: bash

   sudo apt install python3-vcstool python3-colcon-common-extensions # for colcon
   sudo apt install python3-pykdl # for the CRTK Python client library


For cisst/SAW and dVRK, you will also need the following Ubuntu packages:

.. tabs::

   .. tab:: Ubuntu 20.04

      Ubuntu 20.04 with ROS Galactic:

      .. code-block:: bash

         sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev python3-pyudev # dVRK
         sudo apt install ros-galactic-joint-state-publisher* ros-galactic-xacro # ROS

   .. tab:: Ubuntu 22.04

      Ubuntu 20.04 with ROS Humble:

      .. code-block:: bash

         sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev python3-pyudev gfortran-9 # dVRK
         sudo apt install ros-humble-joint-state-publisher* ros-humble-xacro # ROS

Compile cisst/SAW components with ROS dependencies
--------------------------------------------------

Create your ROS 2 workspace and clone all repositories using ``vcs``:

.. code-block:: bash

   source /opt/ros/galactic/setup.bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   vcs import --input https://raw.githubusercontent.com/jhu-dvrk/dvrk_robot_ros2/main/dvrk.vcs --recursive

.. warning:: The URL used as input for ``vcs import`` might be different based on which branches you're using.

.. warning:: If you forgot the ``--recursive`` option, go in ``~/ros2_ws/src/cisst-saw/sawRobotIO1394`` and run ``git submodule init; git submodule update`` (this is to pull the "AmpIO" code).

Compile using ``colcon``:

.. code-block:: bash

   cd ~/ros2_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source ~/ros2_ws/install/setup.bash

.. note:: `colcon build` is not as smart as ``catkin build``: you need
   to be in the top directory of your workspace to build (for example
   ``~/ros2_ws``).  Do not try to build in a sub-directory in your
   workspace, colcon will create a new set of ``build``, ``install``
   and ``log`` directories.  ``catkin build`` recursively look in
   parent directories until it finds the workspace root, ``colcon``
   doesn't.

ROS 2 broadcasts (a lot)
------------------------

By default, ROS2 broadcasts messages based on your network mask
settings.  If you have multiple computers on the same subnet/mask,
they will all share the same "space" by default.  So if you start 2
instances of the dVRK console they will use the same topics, services,
tf names...  This is a bit dangerous as you might be controlling
someone else's robot by accident.  There are multiple ways to handle
this but here are two simple solutions that should cover most cases:

* All your ROS node will be on the same computer and nobody else has
  nodes running on the same computer, use the local host only approach

* Your nodes might be spread on multiple computers or there's a chance
  another user has ROS nodes on any of the computers you're using,
  domain ID will work (as long as no one uses the same ID)

If you use any of the methods below and need to test the results, make
sure you stop and restart the ROS 2 daemon after your
``export``/``unset`` since it will cache some of the discovery
information: ``ros2 daemon stop; ros2 daemon start``

Local host
^^^^^^^^^^

You can set a unique ROS Domain ID , either in your own ``~/.profile``
or for all users with ``/etc/profile.d/ros2.sh``.

.. code-block:: bash

   export ROS_LOCALHOST_ONLY=1

Note that the variable ``ROS_LOCALHOST_ONLY`` just has to be defined.
Setting it to ``0`` doesn't turn this feature off, you would have to
use ``unset`` to disable the local host only broadcast.

Domain ID
^^^^^^^^^

You can set a unique ROS Domain ID , either in your own ``~/.profile``
or for all users with ``/etc/profile.d/ros2.sh``.

.. code-block:: bash

   export ROS_DOMAIN_ID=33

If your organization uses a centralized authentication server (SSO),
one can use the Unix user ID to define the ROS Domain ID.
Unfortunately the domain ID should be between 0 and 101 (see
[ROS_DOMAIN_ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html))
so we can't use the full Unix user Id To automatically set the ROS
Domain ID.  The following configuration file will generate the domain
ID based on the last 2 digits of the user ID.  Create or edit the file
``/etc/profile.d/ros2.sh`` to contain:

.. code-block:: bash

   # set domain id based on last 2 digits of user id
   export ROS_DOMAIN_ID=$(id -u | rev | cut -c 1-2 | rev)

.. warning:: Since this relies on the last two digits of the user ID,
   there is still a strong possibility 2 users will have the same ROS
   Domain ID.  Make sure you run ``ros2 node list`` to check nobody is
   using your domain.

Usage
-----

Example of session
^^^^^^^^^^^^^^^^^^

* Terminal 1: starting the dVRK main console

  * with a real system:

    .. code-block:: bash

       source ~/ros2_ws/install/setup.bash
       cd ~/ros2_ws/install/dvrk_config_jhu # we assume each group has created their own configuration file repository!
       ros2 run dvrk_robot dvrk_console_json -j share/jhu-dVRK-Si/console-PSM1.json

  * with a simulated arm:

    .. code-block:: bash

       source ~/ros2_ws/install/setup.bash
       cd ~/ros2_ws/install/sawIntuitiveResearchKitAll/share/sawIntuitiveResearchKit
       ros2 run dvrk_robot dvrk_console_json -j share/console/console-PSM1_KIN_SIMULATED.json

* Terminal 2: using a Python test script to make the arm move

  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 run dvrk_python dvrk_arm_test.py -a PSM1

* Terminal 3: starting the ROS 2 joint and robot state publishers so we can visualize the arm in RViz

  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 launch dvrk_model dvrk_state_publisher.launch.py arm:=PSM1

* Terminal 4: starting RViz

  .. code-block:: bash

     source ~/ros2_ws/install/setup.bash
     ros2 run rviz2 rviz2 -d ~/ros2_ws/install/dvrk_model/share/dvrk_model/rviz/PSM1.rviz

Note that all the configuration files are installed in the
``ros2_ws/install`` directory during the build so you can
automatically locate them when you write your own ROS launch files.

Useful commands
^^^^^^^^^^^^^^^

* tf2 to pdf: ``ros2 run tf2_tools view_frames`` (then ``evince frames.pdf`` to view)
