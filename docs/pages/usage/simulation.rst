.. _usage-simulation:

**********
Simulation
**********

The patient-side arms can be simulated while the operator uses a physical
console. Start with :ref:`kinematic simulation <usage-kinematic-simulation>`
for client scripts and RViz, or use the :ref:`patient-cart sessions
<usage-simulator-sessions>` below to teleoperate a scene with MTMs or OpenXR.

For first time users, we also have some :ref:`tips<usage-tips>`.

The simulator packages and complete console sessions use ROS 2. Build and
source your workspace as described under :ref:`ROS 2 compilation <ros2>`;
backend installation and runtime requirements are under :ref:`Simulators
<simulators>`.

.. _usage-kinematic-simulation:

Kinematic simulation
####################

This section documents how to run kinematics simulation of da Vinci
Research Kit (dVRK). It is possible to run the dVRK in simulation mode
on any computer as there is no need for a FireWire adapter.

The kinematic simulation mode uses the same software components and
configuration files as the real arm except that the PID controller doesn't
communicate with the hardware. Instead, measured position in the joint space are
based on the last commanded joint values. This mode allows user to test their
code with simple motion commands. In kinematic simulation mode, the dVRK
software retains most of the graphical user interface (IO widgets are not
available of course) as well as the ROS topics. One can use Rviz to visualize
the arms in a virtual environment.

.. warning::

   In simulation mode, there is no way to physically detect which instrument is
   used for the PSM. You need to open the `Arm` `PSM` tab in the GUI and specify
   which instrument is used. Without an instrument, one can not control the PSM
   in cartesian space since the kinematic chain is incomplete.

Run the simulation
******************

Please see :ref:`dVRK system<system>` for more details.  Make sure
you source your ROS workspace's ``setup.bash``.  You can use ROS
launch files for a single arm, a full patient cart or the surgeon's
console (2 MTMs).

.. figure:: /images/gui/ros2-launch-patient-cart-simulated.png
   :width: 600
   :align: center

   dVRK with Classic patient cart simulated in RViz

Remember, the PSMs can only operate if an instrument is selected in the
configuration file, GUI or over ROS topics.

.. code-block:: bash

   # assuming default ROS2 workspace
   source ~/ros2_ws/install/setup.bash

   # console and RViz for a single arm: MTML
   ros2 launch dvrk_model arm.launch.py arm:=MTML generation:=Classic

   # PSM1 Classic
   ros2 launch dvrk_model arm.launch.py arm:=PSM1 generation:=Classic

   # ECM Si
   ros2 launch dvrk_model arm.launch.py arm:=ECM generation:=Si

   # Classic patient cart
   ros2 launch dvrk_model patient_cart.launch.py generation:=Classic

   # S/Si patient cart
   ros2 launch dvrk_model patient_cart.launch.py generation:=Si

   # dVRK only support Classic MTMs so no need to specify generation
   ros2 launch dvrk_model surgeon_console.launch.py

In a second terminal, use a Python test script to make any arm move:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 run dvrk_python dvrk_arm_test.py -a MTML

You can also start investigating which ROS topics are available using:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 topic list

.. warning::

   In kinematic simulation mode, some commands are meaningless since there is no
   IO nor forces measured. The kinematic simulation mode is useful only to test
   simple scripts using position control. There is no simulation of the
   interactions between the robots and their environment.

Configuration and launch files
******************************

The dVRK kinematics simulation uses a simulated low-level hardware class and
reuses the rest of the code including the main ROS node ``dvrk_robot
dvrk_system``, which can run a real or a simulated arm based on a configuration
file in JSON format.

The system example JSON configuration files are located in
|sawIntuitiveResearchKit|_ under ``share/system`` folder. The
simulated arm configuration files are under ``share/arm``.

Usage
*****

Once the simulated dVRK system and RViz are started, you can
interact with the robot using the system's GUI.  Besides "Homing" the
system, you can also use the arm's widget with *direct control*.

You can also communicate with the simulated robot using ROS topics
(see :ref:`dVRK client libraries<devel-ros-clients>`).


.. _usage-simulator-sessions:

Patient-cart sessions with a physical console
#############################################

These examples combine a simulated ECM and PSM1--PSM3 with a real operator
interface. They use different engines to show how the same dVRK console and
teleoperation components connect to either physical MTMs or an OpenXR input
component. The engine controls the virtual arms and renders the endoscope
view; ``dvrk_system`` runs the console and teleoperation components.

.. toctree::
   :maxdepth: 1

   simulation/mtms-newton
   simulation/openxr-pybullet

The examples trace IO, teleoperation, and video separately. Arm commands and
state cross the simulator's ROS 2 CRTK interface. Video reaches the console
over a GStreamer transport. The simulator's robot control is kinematic even
when exercise objects use the engine's physics; see :ref:`Simulators
<simulators>` for backend behavior and limitations.

Other supplied simulator profiles
*********************************

The current launch files also support the following combinations. Each table
entry gives the ROS package followed by its launch filename; run it with
``ros2 launch <package> <filename>``. Choose one session at a time, since the
profiles share arm namespaces and video sockets.

.. list-table::
   :header-rows: 1
   :widths: 16 44 40

   * - Backend
     - Physical MTMs (``dvrk_config_jhu``)
     - OpenXR (``saw_openxr``)
   * - Newton
     - ``davinci_newton.launch.py`` or ``davinci_si_newton.launch.py``
     - ``openxr_newton.launch.py``
   * - PyBullet
     - ``davinci_pybullet.launch.py`` or ``davinci_si_pybullet.launch.py``
     - ``openxr_pybullet.launch.py``
   * - Isaac Sim
     - ``davinci_si_isaac_sim.launch.py``
     - ``openxr_isaac_sim.launch.py``

The ``davinci_si_*`` profiles use the MTMs and Goovis head sensor configured
for the JHU Si setup. They still use simulated patient-side arms. The
``davinci_*`` profiles use the JHU Classic console hardware.

For example, select Newton with OpenXR, or Isaac Sim with OpenXR:

.. code-block:: bash

   ros2 launch saw_openxr openxr_newton.launch.py exercise:=peg_board_ring.yaml rqt:=true

.. code-block:: bash

   ros2 launch saw_openxr openxr_isaac_sim.launch.py exercise:=tray_cubes.yaml rqt:=true

Newton and PyBullet supply Unix-FD stereo to ``stereo_display``, which adds
console status and forwards video to OpenXR. The Isaac Sim OpenXR profile
uses RTSP directly and does not start this Unix-FD overlay process.

These profiles accept ``exercise`` (default ``tray_cubes.yaml``), ``config``
(backend runtime YAML), ``headless`` (default ``true``), and ``rqt`` (default
``false``). ``exercise`` adds an exercise to the profile's patient cart;
``headless:=false`` also opens the simulator viewer. The current shared
launcher leaves homing and teleoperation to the operator.

Source: `dvrk_config_jhu launch files
<https://github.com/jhu-dvrk/dvrk_config_jhu/tree/main/launch>`_,
`sawOpenXR launch files
<https://github.com/adeguet1/sawOpenXR/tree/main/ros/launch>`_, and the
`shared session launcher
<https://github.com/jhu-dvrk/dvrk_simulator_base/blob/main/dvrk_simulator_base/launch.py>`_.

Other dynamic simulators
########################

You can run the dVRK software and emulate the dVRK arms and their environment
using a dynamic simulator. Dynamic simulation for the dVRK has been used for two
main applications:

* Simulate the dynamic of the PSM parallel linkage (da Vinci Standard/Classic).
  This is used mostly to improve control.

* Simulate the interactions between instruments and patient. This can be used to
  simulate surgeries and development training platforms (for humans or ML).
   
The following repositories offer different implementations:

* AMBF: https://github.com/WPI-AIM/ambf
* Isaac Sim: https://github.com/surgical-robotics-ai/isaac-sim-surgical-robotics-challenge
* ORBIT-Surgical: https://orbit-surgical.github.io/
* SurRol: https://github.com/med-air/SurRoL
* CoppeliaSim: https://github.com/marcofer/dynamic-dvrk-coppeliasim-simulator

For any support question, reach out to the developers of each simulator.
