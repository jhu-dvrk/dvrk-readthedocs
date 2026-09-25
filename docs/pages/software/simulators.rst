.. _simulators:
.. _software-simulators:

##########
Simulators
##########

The dVRK simulator suite provides hardware-free simulation backends for the da
Vinci Research Kit. These packages enable software development, algorithm
validation, digital twin visualization, teleoperation practice, and robot
learning without requiring physical patient carts, MTM surgeon consoles, or
FPGA/dVRK controllers.

The simulation ecosystem is organized into four ROS 2 packages:

``dvrk_simulator_base``
  Owns the shared, simulator-independent CRTK ROS 2 contract, message
  validation, command mailboxes, spherical RCM cart frame geometry, and
  scene/exercise schemas.

``dvrk_newton``
  GPU-accelerated kinematics and physics backend built on NVIDIA Newton and
  NVIDIA Warp, featuring sub-millisecond inverse kinematics and multi-arm
  patient cart simulation.

``dvrk_pybullet``
  Lightweight kinematic and dynamic simulation backend powered by PyBullet,
  featuring automatic content-addressed model caching from ``dvrk_model``.

``dvrk_isaac_sim``
  High-fidelity USD-based simulation backend for NVIDIA Isaac Sim 6.0
  (Omniverse Kit), supporting RTX rendering and RTSP stereo streaming.

.. warning::

   These packages support **ROS 2 only**. ROS 1 is not supported.


Architecture
************

The dVRK simulation stack is structured around strict modular decoupling
between CRTK ROS interfaces, physics and kinematics engines, and video frame
transports.

.. figure:: /images/software/dvrk-simulator-architecture.svg
   :width: 700
   :align: center

   dVRK simulation decoupled architecture and data flow


Decoupled design philosophy
===========================

Traditional monolithic simulation integrations tightly bind the simulator's
internal physics engine, asset format, and rendering loop to the robotic
middleware. The dVRK simulation stack departs from this model through three
core architectural separations:

1. **Simulator-independent CRTK ROS surface**: ``dvrk_simulator_base`` provides
   ``ArmRosInterface``, an identical ROS 2 topic graph, message schema, and
   QoS profile across all backends. User applications and client scripts
   interact with simulated arms identically to physical hardware.
2. **Backend-owned URDF and asset ingestion**: The base package describes
   semantic joint and link names, limits, and home positions, but does not parse
   URDF trees or build engine-specific scene graphs. Each backend owns its model
   importer (Warp articulation builder in Newton, URDF loader in PyBullet, USD
   importer in Isaac Sim) and maps joints and links by semantic name.
3. **Decoupled video transport**: High-bandwidth endoscope camera imagery is
   never passed through standard DDS image topics on the critical path. Instead,
   frames travel over zero-copy Linux domain sockets using GStreamer Unix-FD
   shared memory descriptors.


The CRTK ROS 2 contract
=======================

The shared ROS interface defined in ``dvrk_simulator_base.ros_interface.ArmRosInterface``
exposes the standard |CRTK|_ surface:

* **Published topics (All arms)**:
  - Joint state: ``measured_js``, ``setpoint_js``
  - Cartesian state: ``measured_cp``, ``setpoint_cp``
  - Velocity: ``measured_cv``
  - Operating state: ``operating_state``, ``state`` (using transient-local event QoS)
  - Notifications: ``info``, ``warning``, ``error``
* **Published topics (PSMs)**:
  - Jaw state: ``jaw/measured_js``, ``jaw/setpoint_js``
  - Instrument latch: ``tool_type`` (transient-local and latched)
  - Fixed-base Cartesian poses: ``local/measured_cp``, ``local/setpoint_cp``
* **Subscribed topics**:
  - Joint control: ``servo_jp``, ``move_jp``
  - Cartesian control: ``servo_cp``, ``move_cp``
  - Jaw control: ``jaw/servo_jp``, ``jaw/move_jp``
  - State management: ``state_command``

Command queueing semantics
--------------------------

To balance real-time teleoperation with trajectory execution:

* **Servo commands** (``servo_jp``, ``servo_cp``, ``jaw/servo_jp``): Use
  depth-one superseding mailboxes. New setpoints immediately overwrite pending
  ones, guaranteeing minimal latency during teleoperation.
* **Move commands** (``move_jp``, ``move_cp``, ``jaw/move_jp``): Remain
  ordered in a bounded FIFO queue, executing sequentially to completion.
* **State commands** (``state_command``): Processed sequentially to manage
  power and homing transitions deterministically.


Cartesian reference frames and spherical RCMs
=============================================

In a physical dVRK surgical setup, the surgeon views the surgical field through
the endoscope camera mounted on the ECM. Consequently:

* **Active ECM View**: Whenever an ECM arm is present in the scene, each PSM's
  top-level Cartesian topics (``measured_cp``, ``setpoint_cp``, ``servo_cp``,
  ``move_cp``) are automatically evaluated and commanded in the dynamic
  ``ECM_view`` frame. Commands issued by teleoperation or visual servoing
  directly correspond to what is seen on the surgeon's display.
* **Local Frame**: Fixed-base Cartesian coordinates remain available under
  ``<PSM>/local/measured_cp`` and ``<PSM>/local/setpoint_cp`` for low-level
  diagnostics and kinematic verification.

Patient-cart RCM layout generation
----------------------------------

When creating simulation scenes with multiple arms (ECM and PSMs), it can be
tedious and difficult to manually calculate and align 3D base frames so that each
manipulator's Remote Center of Motion (RCM) properly targets a common surgical
anatomy.

To simplify this, ``dvrk_simulator_base`` provides a geometry tool to compute
base frames assuming all arms are positioned on a virtual sphere focused toward
the surgical target. Arm positions and orientations are controlled using
intuitive azimuth and polar coordinates.

.. note::

   Using this spherical layout tool is **not a requirement**; arbitrary base
   frames can always be defined directly in your scene YAML files. It is simply
   a convenient tool to simplify and speed up scene creation across all dVRK
   simulators.

The tool is available as both a CLI generator and an interactive visual editor:

* **Command-line generator**:

  .. code-block:: bash

     # Output default patient-cart frames YAML
     ros2 run dvrk_simulator_base generate_cart_frames

     # Update an existing scene YAML in place
     ros2 run dvrk_simulator_base generate_cart_frames -s scene.yaml

* **Interactive PyQt6 editor**:

  .. code-block:: bash

     ros2 run dvrk_simulator_base cart_frame_editor

  Provides interactive azimuth and polar angle sliders for all arms, top-down
  and lateral layout previews, and a copyable YAML configuration snippet.


Separation of video and control data
====================================

Stereo camera rendering at 60 or 120 FPS generates massive pixel throughput.
Routing uncompressed images through standard ROS middleware causes CPU cache
thrashing, memory allocation overhead, and variable queue latency.

To ensure sub-frame teleoperation latency:

* Simulated cameras render directly into Linux shared memory file descriptors
  (``memfd``).
* Buffers are emitted through Linux domain abstract sockets using the
  GStreamer ``unixfdsink`` element under the canonical convention:

  .. code-block:: text

     @dvrk:<backend>:stereo_source   (e.g., @dvrk:newton:stereo_source, @dvrk:pybullet:stereo_source)

* Consumers (such as ``dvrk_console stereo_display`` or VR headset bridges)
  connect via ``unixfdsrc`` using one-frame leaky queues
  (``leaky=downstream``). Old frames are automatically discarded if a receiver
  falls behind, preventing display lag.


Thread isolation and execution pacing
=====================================

All backends enforce strict concurrency isolation between ROS 2 callbacks and
the simulation engine:

* ROS callbacks only validate incoming message fields and push commands into
  thread-safe lock-free mailboxes.
* The backend physics and kinematics runtime ticks on an independent thread
  at a steady simulation rate (e.g. 120 Hz GPU loop in Newton, or fixed-step
  in PyBullet).
* Model states are published from the engine thread back to ROS, eliminating
  race conditions and middleware-induced physics jitter.

.. _dvrk_simulator_base:

dvrk_simulator_base
===================

``dvrk_simulator_base`` contains shared contracts, utilities, and configuration
assets common to all simulator backends. It has zero external dependencies on
physics engines or proprietary rendering libraries.

Key components:

* ``ArmRosInterface``: Shared CRTK ROS 2 publisher/subscriber implementation.
* **Cart frame tools**: ``generate_cart_frames`` and ``cart_frame_editor``.
* **System startup tool**: ``start_dvrk_system`` automatically sequences
  system power, homing, and console state transitions.
* **Shared assets and scenes**: Located in ``share/scenes/`` (e.g.
  ``ECM_PSM1_PSM2.yaml``, ``ECM_PSM1_PSM2_PSM3.yaml``) and ``share/exercises/``
  (e.g. ``tray_cubes.yaml``, ``peg_board_ring.yaml``, ``peg_board_CUHK.yaml``).


Instances
*********

The dVRK ecosystem provides 3 distinct simulators based on:
- Newton (GPU accelerated, high performance)
- PyBullet (Lightweight, CPU-based)
- Isaac Sim (High-fidelity, USD-based)

.. _dvrk_newton:

dvrk_newton
===========

``dvrk_newton`` is the GPU-accelerated simulation backend built with NVIDIA
Newton (``newton``) and NVIDIA Warp (``warp-lang``). It executes kinematics and
physics kernels directly on CUDA-enabled NVIDIA GPUs.

Features:

* **GPU Kinematics**: Forward kinematics (``newton.eval_fk``) evaluated directly
  on GPU in under 0.5 ms.
* **Damped Least-Squares IK**: High-speed numerical inverse kinematics
  converging in real time with sub-micrometer precision.
* **Mimic Joint Support**: Automatically parses and constrains dVRK mimic joints
  (e.g. PSM ``jaw_1`` and ``jaw_2`` mirroring ``jaw``) within Warp articulation
  graphs.
* **Multi-Arm Scenes**: Simulates full patient carts (ECM + 3 PSMs) within a
  single unified simulation world.

Environment setup
-----------------

Because Newton and Warp require specific GPU runtime libraries, dependencies are
managed via a dedicated virtual environment:

.. code-block:: bash

   # Bootstrap .venv-newton with system site packages
   ./src/dvrk/dvrk_newton/scripts/bootstrap_venv.sh -y

   # Build the ROS package
   colcon build --symlink-install --packages-select dvrk_newton
   source install/setup.bash

Running scenes
--------------

.. tabs::

   .. tab:: Single Arm (PSM1)

      .. code-block:: bash

         # Interactive ViewerGL window
         ros2 launch dvrk_newton simulator.launch.py model:=PSM1

         # Headless mode (no GUI window)
         ros2 launch dvrk_newton simulator.launch.py model:=PSM1 headless:=true

         # Launch with rqt monitor
         ros2 launch dvrk_newton simulator.launch.py model:=PSM1 rqt:=true

   .. tab:: Full Patient Cart

      .. code-block:: bash

         ros2 launch dvrk_newton simulator.launch.py scene:=ECM_PSM1_PSM2_PSM3.yaml


.. _dvrk_pybullet:

dvrk_pybullet
=============

``dvrk_pybullet`` provides a lightweight, CPU- and EGL-compatible simulation
backend powered by PyBullet. It requires no specialized GPU drivers and is ideal
for CI/CD testing, headless servers, and rapid algorithm iteration.

Features:

* **Model Generation Cache**: Converts Xacro descriptions from ``dvrk_model``
  into resolved absolute URDF models stored under content-addressed hashes in
  ``~/.cache/dvrk_pybullet/<hash>/``.
* **Kinematic Control**: Setpoints are enforced with kinematic joint resets,
  bypassing PID tuning while maintaining mimic joint constraints.
* **Unix-FD Video Output**: Exports mono (``@dvrk:pybullet:mono_source``) and
  side-by-side stereo (``@dvrk:pybullet:stereo_source``) camera frames via
  Unix-FD.

Environment setup
-----------------

.. code-block:: bash

   # Bootstrap .venv-pybullet
   ./src/dvrk/dvrk_pybullet/scripts/bootstrap_venv.sh -y

   # Build the ROS package
   colcon build --symlink-install --packages-select dvrk_simulator_base dvrk_pybullet
   source install/setup.bash

Running scenes
--------------

.. tabs::

   .. tab:: Model Preview

      Quickly verify robot models and instruments at configured home poses:

      .. code-block:: bash

         ros2 run dvrk_pybullet dvrk_pybullet_preview --model PSM1 --instrument 420006

   .. tab:: Multi-Arm Scene

      .. code-block:: bash

         ros2 launch dvrk_pybullet simulator.launch.py scene:=ECM_PSM1_PSM2.yaml rqt:=true

   .. tab:: GStreamer Camera View

      Connect to the simulated endoscope camera stream from a separate terminal:

      .. code-block:: bash

         gst-launch-1.0 unixfdsrc socket-path=dvrk:pybullet:mono_source \
           socket-type=abstract \
           ! queue leaky=downstream max-size-buffers=1 \
           ! videoconvert ! autovideosink sync=false


.. _dvrk_isaac_sim:

dvrk_isaac_sim
==============

``dvrk_isaac_sim`` integrates dVRK with NVIDIA Isaac Sim 6.0 (Omniverse Kit),
delivering photorealistic RTX rendering, physically based materials, and
Omniverse digital twin pipelines.

Features:

* **Canonical Robot Assets**: Loads meshes and kinematic trees from
  ``dvrk_arm_description`` and ``dvrk_model``.
* **Dual Video Pipeline**: Publishes standard ROS 2 ``sensor_msgs/msg/Image``
  topics (``/ECM/image_raw``) alongside low-latency H.264 RTSP streams
  (``rtsp://localhost:8554/ECM``).
* **Dockable Monitoring**: Integrates with ``rqt_dvrk`` for multi-arm and
  diagnostics monitoring.

Environment setup
-----------------

Isaac Sim requires Ubuntu 24.04, ROS 2 Jazzy, and Isaac Sim 6.0:

.. code-block:: bash

   export ISAAC_SIM_DIR=/path/to/isaac-sim
   colcon build --symlink-install --packages-select dvrk_isaac_sim
   source install/setup.bash

Running scenes
--------------

.. code-block:: bash

   # Run stereo patient cart scene
   ros2 launch dvrk_isaac_sim simulator.launch.py \
     scene:=ECM_PSM1_PSM2_PSM3_stereo.yaml rqt:=true


Backend comparison matrix
=========================

.. list-table::
   :widths: 18 18 18 22 24
   :header-rows: 1

   * - Package
     - Physics / Kinematics
     - Acceleration
     - Camera Transport
     - Primary Strengths
   * - ``dvrk_newton``
     - NVIDIA Newton & Warp
     - NVIDIA GPU (CUDA)
     - Unix-FD abstract socket
     - Real-time 120 Hz GPU loop, fast DLS IK (<0.5 ms), VR teleoperation
   * - ``dvrk_pybullet``
     - PyBullet
     - CPU or GPU (EGL/Tiny)
     - Unix-FD abstract socket
     - Lightweight, zero GPU driver requirement, fast headless CI/CD testing
   * - ``dvrk_isaac_sim``
     - Isaac Sim 6.0 (PhysX)
     - NVIDIA RTX GPU
     - ROS Image & RTSP (H.264)
     - Photorealistic RTX ray-tracing, Omniverse USD ecosystem
   * - ``dvrk_simulator_base``
     - Contract / Interface
     - N/A
     - N/A
     - Shared CRTK ROS 2 layer, RCM geometry math, scene/exercise schemas


Example: Newton and OpenXR
**************************

This example demonstrates a complete virtual reality teleoperation workstation.
An operator wearing a Meta Quest headset controls the simulated 4-arm patient
cart running in ``dvrk_newton``, views the 3D surgical scene through the
simulated stereo endoscope with dVRK head-up display (HUD) overlays, and
manipulates objects in interactive training exercises.

.. figure:: /images/software/dvrk-newton-openxr.svg
   :width: 700
   :align: center

   Teleoperation pipeline connecting Meta Quest (sawOpenXR) to dvrk_newton


Prerequisites
=============

1. **dVRK Packages**: Ensure ``dvrk_newton``, ``dvrk_simulator_base``, and
   ``dvrk_console`` are built in your workspace.
2. **External Packages**:

   * `sawOpenXR <https://github.com/adeguet1/sawOpenXR>`_: Provides the OpenXR
     driver, video ingestion, and controller tracking.
   * `sawIntuitiveResearchKit <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_:
     Provides ``dvrk_system`` and CRTK teleoperation components.

3. **Headset Setup**: Connect your Meta Quest to the workstation via ALVR,
   WiVRn, or Meta Quest Link, and ensure OpenXR is active.



Running the launch file
=======================

Launch the integrated simulation, dVRK console video overlay, and teleoperation
manager with a single command:

.. code-block:: bash

   # Activate the Newton environment and source ROS 2 workspace
   source ~/ros2_ws/.venv-newton/bin/activate
   source ~/ros2_ws/install/setup.bash

   # Launch patient cart, console overlay, and OpenXR teleoperation
   ros2 launch dvrk_newton open_xr.launch.py

What happens during startup:

1. **Newton Simulation**: Starts ``dvrk_newton`` with the four-arm patient cart
   (``ECM_PSM1_PSM2_PSM3.yaml``) and overlays the default ``tray_cubes.yaml``
   exercise scene. It publishes raw stereo video to ``@dvrk:simulator:stereo_source``.
2. **Video Overlay**: ``dvrk_console stereo_display`` attaches to the raw stream,
   renders the real-time vector HUD (status messages, clutch and camera icons),
   and exposes ``@dvrk:console:stereo_overlay``.
3. **OpenXR Bridge**: ``sawOpenXR`` receives the composited overlay stream and
   projects it into the headset display.
4. **Teleoperation**: ``dvrk_system`` maps Quest controller poses to MTML and
   MTMR, initiating teleoperation with PSM1 and PSM2.
5. **System Startup**: ``start_dvrk_system`` powers on and homes the simulated
   arms automatically.


Launch arguments and options
============================

The ``open_xr.launch.py`` launch file accepts several configurable parameters:

.. list-table::
   :widths: 20 20 60
   :header-rows: 1

   * - Argument
     - Default
     - Description
   * - ``scene``
     - ``tray_cubes.yaml``
     - Exercise scene YAML file or installed exercise name (e.g. ``peg_board_ring.yaml``, ``peg_board_CUHK.yaml``).
   * - ``headless``
     - ``true``
     - When ``true``, disables the desktop OpenGL window since the HMD provides visual output. Set to ``false`` to also display a desktop window.
   * - ``rqt``
     - ``false``
     - When ``true``, opens the dockable dVRK Console, CRTK Arms tabs, and diagnostics monitor.
   * - ``console``
     - ``console``
     - ROS namespace for the dVRK console node.

Selecting alternative training exercises
----------------------------------------

To load the peg-board ring transfer exercise with an accompanying desktop
viewer:

.. code-block:: bash

   ros2 launch dvrk_newton open_xr.launch.py \
     scene:=peg_board_ring.yaml \
     headless:=false

Enabling the rqt monitor
------------------------

To inspect joint kinematics, Cartesian positions, and system events while
operating:

.. code-block:: bash

   ros2 launch dvrk_newton open_xr.launch.py rqt:=true


Teleoperation and operator controls
===================================

When the system is running:

* **Manipulator Control**: Move the right controller to move PSM1; move the
  left controller to move PSM2. Squeeze the index triggers to actuate the
  instrument jaws.
* **Clutching**: Press and hold the controller grip button to disengage
  teleoperation, allowing you to reposition your hands into an ergonomic
  posture without moving the surgical instruments.
* **Camera Control**: Press the secondary controller button (or configured
  camera pedal) to transition MTM control from the PSMs to the ECM, allowing
  you to reposition and zoom the endoscopic camera view.
