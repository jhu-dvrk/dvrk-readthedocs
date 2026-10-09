.. _simulators:
.. _software-simulators:

##########
Simulators
##########

The dVRK simulation packages expose ROS 2 CRTK interfaces for virtual PSMs and
ECMs. Use them to develop clients, exercise Cartesian control, and test
teleoperation with a configured patient cart. Robot joint names, limits, and
home positions come from ``dvrk_arm_description``; meshes and virtual URDFs come
from ``dvrk_model``.

``dvrk_simulator_base`` provides the common ROS interface, command controller,
scene loading, and launch helpers. ``dvrk_newton``, ``dvrk_pybullet``, and
``dvrk_isaac_sim`` provide the engine integration. Their robot control is
kinematic: simulated instrument motion does not reproduce hardware servo
control or full patient-cart mechanics. Exercise objects and grasp behavior
also differ between engines.

These packages support **ROS 2 only**. The examples below use a sourced ROS 2
Jazzy workspace on Ubuntu 24.04 and run from its root directory.


Choosing a backend
******************

.. list-table::
   :widths: 20 30 25 25
   :header-rows: 1

   * - Package
     - Robot and exercise behavior
     - Rendering / video
     - Typical use
   * - ``dvrk_newton``
     - Newton/Warp world; kinematic arms; pose-error grasp attachments
     - NVIDIA GPU camera rendering; Unix-FD video
     - Patient-cart exercises and OpenXR teleoperation
   * - ``dvrk_pybullet``
     - PyBullet world; kinematic joint resets; pose-error or force/torque grasp policies
     - EGL or Tiny rendering; Unix-FD video
     - CPU simulation and headless integration tests
   * - ``dvrk_isaac_sim``
     - Isaac Sim 6.0; visual kinematic PSMs; imported exercise objects; no grasp attachments
     - RTX rendering; native RTSP mono or side-by-side stereo
     - USD scenes and endoscope visualization
   * - ``dvrk_simulator_base``
     - Shared contracts, command handling, scene composition, and frame utilities
     - No simulator engine or camera renderer
     - Common behavior across all three backends

PyBullet's ``tiny`` renderer is the CPU option; ``egl`` needs a working EGL
configuration. Newton supports ``device:=cpu`` for compatible simulation and
integration tests, but this does not provide its GPU camera rendering.


Shared runtime and ROS interfaces
*********************************

.. figure:: /images/software/dvrk-simulator-architecture.svg
   :width: 700
   :align: center
   :alt: ROS frontend and separate simulator worker connected by private Unix-socket IPC, with video on a separate transport.

   All three backends use the same ROS frontend / simulator worker boundary.

A launch starts the frontend with **ROS Python**. The frontend starts a separate
worker with the selected engine interpreter. Newton, Warp, PyBullet, and Isaac
are not imported into the ROS frontend. Both interpreters must be able to
import the built dVRK packages.

The private Unix socket carries validated commands, complete-scene snapshots,
operating-state events, warnings, and metrics. Writes are bounded and
nonblocking; an unsent old snapshot can be replaced by a newer one. State events
remain ordered. Video uses a separate transport and does not cross this socket.

The simulator worker owns command execution, trajectories, reference-frame
conversion, FK/IK, and engine state. ROS callbacks validate messages and enqueue
commands; the ROS executor publishes completed snapshots. Command mailboxes
are bounded and protected by a lock.

CRTK topics and command behavior
================================

Each configured arm exposes the common |CRTK|_ interface under its arm name,
for example ``/PSM1/measured_js``:

* State: ``measured_js``, ``setpoint_js``, ``measured_cp``, ``setpoint_cp``,
  ``measured_cv``, ``operating_state``, and ``state``.
* Notifications: ``info``, ``warning``, and ``error``.
* Commands: ``servo_jp``, ``move_jp``, ``servo_cp``, ``move_cp``, and
  ``state_command``.
* PSM additions: ``jaw/measured_js``, ``jaw/setpoint_js``, ``jaw/servo_jp``,
  ``jaw/move_jp``, ``tool_type``, ``local/measured_cp``, and ``local/setpoint_cp``.

Operating-state events and ``tool_type`` use transient-local QoS.

The newest pending arm motion replaces any older pending arm motion, including
across servo and move commands. Jaw motion has its own superseding mailbox.
**Move commands are not queued to execute sequentially to completion.** A new
motion command can replace an active trajectory. State commands use a bounded
FIFO, controlled by ``command_queue_capacity``.

Moves use synchronized, velocity-limited joint trajectories. An arm servo does
not cancel a jaw move. Disabling, pausing, or unhoming an arm cancels its active
motion and rejects new motion until its operating state permits it. Invalid
commands produce warnings without applying the rejected target.

Cartesian reference frames
==========================

When an ECM is configured, PSM top-level Cartesian poses use ``ECM_view`` by
default. PSM ``local/*`` poses remain in the configured PSM base frame. Cartesian
commands retain their original ``frame_id`` until the worker resolves them;
explicit world/parent and arm-base frames are also supported.

Commands consumed in a step use the ECM pose from the preceding completed
scene. An ECM motion command in that same step changes the reference for the
next step. Published Cartesian values come from one completed scene snapshot.

Monitoring
==========

Use ``rqt:=true`` to open the Arms and Diagnostics monitor. For a generic
simulator launch, add ``rqt_console:=true`` to include Console monitoring when a
dVRK console is running. Patient-cart and OpenXR profiles include Console
monitoring automatically when rqt is enabled.

All backends report five metrics on ``/diagnostics``:

* ``simulation_hz``: completed simulation steps per wall-clock second.
* ``camera_hz``: camera frames delivered to the video sink per second; zero when disabled.
* ``state_publish_hz``: completed ROS scene publication cycles per second.
* ``snapshot_receive_hz``: complete-scene snapshots received by the ROS frontend per second.
* ``snapshot_age_ms``: age of the latest received snapshot, including IPC transit time.

These are measured rates. Setting a simulation or camera rate does not guarantee
that the engine or GPU will sustain it.


Configuration and scene composition
***********************************

Keep backend settings in a runtime YAML and robot/camera/object settings in
scene YAML. Unknown runtime and grasp options are rejected. Rates must be
positive and finite; booleans must be YAML ``true`` or ``false``.

Common runtime fields
=====================

.. list-table::
   :widths: 30 20 50
   :header-rows: 1

   * - Field
     - Default
     - Meaning
   * - ``headless``
     - ``false``
     - Disable the desktop viewer. Camera rendering is configured separately.
   * - ``simulation_rate_hz``
     - ``120.0``
     - Target simulation step rate.
   * - ``state_publish_rate_hz``
     - ``100.0``
     - ROS frontend publication rate.
   * - ``command_queue_capacity``
     - ``32``
     - Capacity of the ordered state-command queue.
   * - ``generated_root``
     - ``null``
     - Use the backend's user cache; an explicit relative path is resolved against the runtime YAML directory.
   * - ``scene``
     - ``null``
     - Optional default scene when no launch/CLI scene is supplied.

These are class defaults; a shipped profile may specify different values.
Backend-specific fields are described below.

Launch arguments
================

``simulator.launch.py`` accepts ``config``, ``scene``, ``headless``, ``rqt``,
``rqt_console``, and ``console``. Newton also accepts ``device``.

A generic launch leaves ``headless`` unset, so the runtime YAML decides whether
to open a desktop viewer. An explicit ``headless:=true`` or ``headless:=false``
overrides it. Patient-cart and OpenXR profiles default to ``headless:=true``.
``console`` selects the namespace used by startup and monitoring clients; it
does not rewrite the system JSON or video-overlay configuration.

The shared launch helpers construct sessions and arrange shutdown when a
supervised simulator, dVRK system, or display process exits. Backend launch
filenames remain the entry points for users.

Scenes and exercises
====================

Bare scene filenames are searched in the backend's installed scenes, shared
``dvrk_simulator_base`` scenes/exercises, and runtime-config scene directories.
Absolute paths and explicit relative paths are also supported. Arm YAML names
such as ``PSM1.yaml`` resolve through ``dvrk_arm_description``.

A camera-free single-arm scene can be as small as:

.. code-block:: yaml

   scene:
     name: single_psm
     robots:
       - config: PSM1.yaml
         instrument: "420006"

Save it as ``single_psm.yaml`` and pass its absolute path to ``scene``. There
is no separate ``model`` launch argument or PyBullet preview executable.

A scene can include a cart and an exercise:

.. code-block:: yaml

   scene:
     name: cart_with_tray
     include:
       - ECM_PSM1_PSM2_PSM3.yaml
       - tray_cubes.yaml

Includes are loaded before the including scene. Shared includes are loaded
once; circular includes are rejected. Later camera settings override earlier
ones, including ``mode: "off"``. Quote ``"off"`` because YAML can otherwise
interpret it as a boolean. Avoid declaring the same robot or object twice.
For Isaac, use ``ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml`` as the cart to select its
supported RTSP camera settings.

The frontend CLI also accepts repeated ``--scene`` arguments:

.. code-block:: bash

   ros2 run dvrk_pybullet simulator_node \
     --scene ECM_PSM1_PSM2_PSM3.yaml --scene tray_cubes.yaml --headless true

JHU patient-cart scene files include a platform's ``simulator_cart.yaml`` and
add backend camera settings. Identical system configurations share a common
JSON through relative symlinks; backend differences are kept separately.


.. _dvrk_simulator_base:

dvrk_simulator_base
*******************

The base package provides shared configuration and launch construction,
``ArmRosInterface``, ``SimulatorRosNode``, worker IPC, and ``ArmController``.
The controller owns operating state and independent arm/jaw trajectories.
Backends retain their IK implementation and engine integration. Newton and
Isaac share a URDF FK/Jacobian evaluator; engine-specific model import remains
in each backend. The base package does not depend on simulator engines.

Shared scenes and exercises include ``ECM_PSM1_PSM2.yaml``,
``ECM_PSM1_PSM2_PSM3.yaml``, ``tray_cubes.yaml``, ``peg_board_ring.yaml``, and
``peg_board_CUHK.yaml``. Exercise aliases include their canonical scene files.

Patient-cart frames
===================

Generate RCM/base poses on a sphere directed toward a common surgical target:

.. code-block:: bash

   ros2 run dvrk_simulator_base generate_cart_frames
   ros2 run dvrk_simulator_base generate_cart_frames -s scene.yaml
   ros2 run dvrk_simulator_base cart_frame_editor

The editor requires PyQt6 and provides top/side previews and adjustable arm
azimuth/polar coordinates. PyQt6 is optional for the other tools. Arbitrary
base poses can also be written directly in ``scene.frames``.

System startup
==============

``ros2 run dvrk_simulator_base start_dvrk_system`` waits for system/console
connections, requests homing, and enables teleoperation after its configured
wait. Use ``--no-teleop`` to request homing only. OpenXR and 3Dconnexion profiles
start this helper automatically. JHU profiles with a control panel leave homing
and teleoperation to the operator.


.. _dvrk_pybullet:

dvrk_pybullet
*************

PyBullet runs all configured arms in a shared world. Kinematic joint resets
apply the commanded positions and mimic joints. Dynamic exercise objects and
grasp constraints remain engine-owned. The physics timestep follows
``simulation_rate_hz``. Camera rendering runs in a separate process with its
own PyBullet connection.

Setup and launch
================

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   ./src/dvrk/dvrk_pybullet/scripts/bootstrap_venv.sh -y
   colcon build --symlink-install --packages-up-to dvrk_pybullet
   source install/setup.bash
   ros2 launch dvrk_pybullet simulator.launch.py \
     scene:=ECM_PSM1_PSM2.yaml headless:=true rqt:=true

The bootstrap creates ``.venv-pybullet`` with system site packages. To select
another interpreter, set ``DVRK_PYBULLET_PYTHON`` to its Python executable. The
selected interpreter is cached under ``~/.cache/dvrk_pybullet``. Activating the
engine environment in the ROS launch shell is unnecessary.

Backend settings and video
==========================

Set ``renderer: egl`` or ``renderer: tiny`` in the **runtime** YAML. A
``scene.camera.renderer`` setting is rejected. The ``grasp`` mapping supports
``policy: pose_error`` or ``policy: force_torque``, per-arm policies under
``grasp.arms``, grasp markers, contact qualification, load thresholds, and
constraint tuning. Use the shipped ``share/pybullet.yaml`` as a complete example.

Generated URDFs and metadata use content-addressed directories under
``~/.cache/dvrk_pybullet/<content-hash>/``. Camera scenes select Unix-FD output
and socket names. The shared mono scene can be viewed with:

.. code-block:: bash

   gst-launch-1.0 unixfdsrc socket-path=dvrk:simulator:mono_source \
     socket-type=abstract do-timestamp=true \
     ! queue leaky=downstream max-size-buffers=1 \
     ! videoconvert ! autovideosink sync=false

Unix-FD video requires GStreamer's ``unixfdsrc`` / ``unixfdsink`` elements.
For a bounded scene test:

.. code-block:: bash

   ros2 launch dvrk_pybullet test_scene.launch.py \
     scene:=ECM_PSM1_PSM2.yaml timeout:=5.0


.. _dvrk_newton:

dvrk_newton
***********

Newton uses NVIDIA Newton and Warp for its world and physics integration.
Cartesian commands use a CPU URDF analytic Jacobian when startup checks agree
with Newton FK. If those checks fail, the arm falls back to Newton FK-based
numerical IK and reports a warning. The camera worker consumes the latest
completed poses; simulation and rendering still compete for GPU resources.

Setup and launch
================

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   ./src/dvrk/dvrk_newton/scripts/bootstrap_venv.sh -y
   colcon build --symlink-install --packages-up-to dvrk_newton
   source install/setup.bash
   ros2 launch dvrk_newton simulator.launch.py \
     scene:=ECM_PSM1_PSM2_PSM3.yaml headless:=true rqt:=true

The bootstrap creates ``.venv-newton``. Set ``DVRK_NEWTON_PYTHON`` to choose
another engine interpreter. Its selection is cached under
``~/.cache/dvrk_newton``.

Backend settings and grasping
=============================

The runtime adds ``device`` (default ``cuda:0``), ``rigid_gap_m`` (default
``0.005``), and ``grasp``. Newton's grasping uses pose-error qualification and
release thresholds. Supported fields are ``max_grasps_per_object``,
``close_threshold_rad``, ``release_threshold_rad``, ``break_distance_m``,
``break_orientation_rad``, ``contact_region_offset_m``, and
``contact_region_radius_m``. A maximum of zero means unlimited grasps per object.

PyBullet's force/torque policies, per-arm policies, grasp markers, force limits,
and constraint ERP settings are not Newton options. They are rejected instead
of silently ignored. Newton exports camera video over Unix-FD; its OpenXR
profile sets the shared stereo socket used by the console overlay.


.. _dvrk_isaac_sim:

dvrk_isaac_sim
**************

Isaac Sim 6.0 renders visual kinematic PSMs and imports exercise URDF/USD
objects. The ECM supplies the camera frame without a visible endoscope mesh.
Missing PSM assets are converted inside the worker's existing Isaac application.
This backend does not implement Newton/PyBullet grasp attachments.

Setup and launch
================

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --packages-up-to dvrk_isaac_sim
   source install/setup.bash
   export ISAAC_SIM_DIR=/path/to/isaac-sim
   ros2 launch dvrk_isaac_sim simulator.launch.py \
     scene:=ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml headless:=true rqt:=true

Interpreter selection happens at **runtime**, not during the package build.
Use ``DVRK_ISAAC_SIM_PYTHON`` to select the executable explicitly, or
``ISAAC_SIM_DIR`` to locate an installation's ``python.sh``. A valid cached
selection takes precedence over installation discovery. Generated assets and
interpreter selection use ``$XDG_CACHE_HOME/dvrk_isaac_sim`` (normally
``~/.cache/dvrk_isaac_sim``), unless ``generated_root`` overrides it.

Backend settings and video
==========================

The runtime adds ``render_rate_hz`` (default ``30.0``) and ``renderer``. Accepted
renderers are ``MinimalRendering``, ``RaytracedLighting`` (default),
``RealTimePathTracing``, and ``PathTracing``. Simulation and rendering are
scheduled independently. Camera poses are applied before rendering and frames
are extracted afterwards.

The current worker uses native **RTSP only**, including tiled side-by-side
stereo. It does not publish ROS image topics or implement Unix-FD camera
output. Select an Isaac scene with ``transports: [rtsp]``; the supplied scenes
use ``rtsp://localhost:8554/ECM``. The scene's ``rtsp.encoding`` selects its
video pipeline; the removed top-level ``camera.encoding`` is not that setting.

.. code-block:: bash

   ros2 launch dvrk_isaac_sim test_scene.launch.py \
     scene:=ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml timeout:=5.0
   ros2 run dvrk_isaac_sim clean_cache

``clean_cache`` removes recognized converted assets while keeping interpreter
selection and unrelated files. Use ``--config /path/to/runtime.yaml`` or
``--generated-root /path/to/cache`` for a custom asset directory.

Older ``isaac_sim_dir``, ROS middleware, and ``generated_dir`` runtime fields
belonged to the in-process adapter. Use the interpreter environment variables
and ``generated_root`` with the current frontend/worker architecture.


OpenXR and patient-cart sessions
********************************

Each backend supplies ``open_xr.launch.py``. These profiles combine a patient
cart with ``tray_cubes.yaml`` by default, start ``dvrk_system`` with the supplied
OpenXR system JSON, and request homing/teleoperation through
``start_dvrk_system``. Build the required ``dvrk_robot``, ``dvrk_console``, and
``saw_openxr`` packages and configure a working OpenXR runtime for the headset.

.. figure:: /images/software/dvrk-newton-openxr.svg
   :width: 700
   :align: center
   :alt: Newton worker video reaches the console overlay and OpenXR headset while ROS CRTK commands pass through dvrk_system and the ROS frontend.

   Newton's OpenXR profile keeps ROS control and Unix-FD video separate.

.. code-block:: bash

   source install/setup.bash
   ros2 launch dvrk_newton open_xr.launch.py rqt:=true
   ros2 launch dvrk_pybullet open_xr.launch.py scene:=peg_board_ring.yaml
   ros2 launch dvrk_isaac_sim open_xr.launch.py scene:=tray_cubes.yaml

Run one session at a time. Newton and PyBullet profiles send raw stereo to
``@dvrk:simulator:stereo_source``. ``dvrk_console stereo_display`` adds the
console overlay and supplies ``@dvrk:console:stereo_overlay`` to sawOpenXR.
Isaac's profile uses RTSP directly and does not launch this Unix-FD overlay.
Socket names and RTSP endpoints are profile/scene settings, not universal
backend defaults.

Use ``headless:=false`` to open a desktop viewer as well. ``scene`` selects an
exercise added to the profile's cart; ``config`` selects runtime settings.
``console`` configures monitoring/startup clients. Operator controls and
teleoperation mappings are defined by the installed sawOpenXR and system
configuration.

JHU profiles are launched from ``dvrk_config_jhu``, for example:

.. code-block:: bash

   ros2 launch dvrk_config_jhu daVinciSi_MTML_MTMR_newton.launch.py \
     exercise:=tray_cubes.yaml rqt:=true

They include the control panel and leave homing and teleoperation to the
operator. The Si launch variants are ``daVinciSi_MTML_MTMR_pybullet.launch.py``
and ``daVinciSi_MTML_MTMR_isaac_sim.launch.py``. Classic-cart variants are
``daVinci_MTML_MTMR_newton.launch.py`` and
``daVinci_MTML_MTMR_pybullet.launch.py``.
