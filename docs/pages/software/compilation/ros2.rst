.. _ros2:

*****
ROS 2
*****

This code hasn't been tested extensively.  We welcome any feedback.
The following has been tested on Ubuntu 20.04 with ROS 2 Galactic and
Ubuntu 22.04 with ROS 2 Humble.

Ubuntu packages
###############

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

      Ubuntu 22.04 with ROS Humble:

      .. code-block:: bash

         sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion libcppunit-dev libqt5xmlpatterns5-dev libbluetooth-dev python3-pyudev gfortran-9 # dVRK
         sudo apt install ros-humble-joint-state-publisher* ros-humble-xacro # ROS

Colcon workspace, clone and build
#################################

Create your ROS 2 workspace and clone all repositories using ``vcs``:

.. code-block:: bash

   # this depends on the ROS version you're using
   source /opt/ros/galactic/setup.bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # make sure you use the correct vcs file
   vcs import --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros2-dvrk-2.3.0.vcs --recursive

To use the development branches, replace the vcs line with:

.. code-block:: bash

   vcs import --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros2-dvrk-devel.vcs --recursive

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

Testing
#######

Environment variables
*********************

If you have a single ros2 workspace for a given user account, you
might want to automatically ``source`` the ``setup.bash`` when you log
in. To do so, you should add the following lines at the end of your
``~/.bashrc`` (hidden file in your home directory):

.. code-block:: bash

   # for ROS
   if [ -f ~/ros2_ws/install/setup.bash ]; then
     . ~/ros2_ws/install/setup.bash
   fi

.. note::

   There is no need to source ``cisstvars.sh`` for ROS 2.  It is added
   as a hook in the *cisst* ``colcon.pkg`` file.

Examples
********

Use a ROS launch file to start the dVRK in simulated mode with the patient cart only:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 launch dvrk_model patient_cart.launch generation:=Classic

.. figure:: /images/gui/ros2-launch-patient-cart-simulated.png
   :width: 600
   :align: center

   dVRK with Classic patient cart simulated in RViz

In a second terminal, use a Python test script to make an arm move

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run dvrk_python dvrk_arm_test.py -a PSM1

The main dVRK ROS node is ``dvrk_console_json`` from the
``dvrk_robot``.  This node requires configuration files specific to
your system.

Notes
*****

* ROS2 will broadcast on your subnet.  That means that other computers
  on the same subnet might send ROS messages you don't want.  To
  prevent this, google ROS_LOCALHOST_ONLY or ROS_DOMAIN_ID.
