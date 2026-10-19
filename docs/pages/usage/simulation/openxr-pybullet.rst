.. _usage-openxr-pybullet:

********************
OpenXR with PyBullet
********************

This example uses an OpenXR headset and tracked controllers as the operator
console for a simulated ECM and PSM1--PSM3. ``sawOpenXR`` supplies the
``MTML``/``MTMR`` interfaces and console input events. PyBullet supplies
patient-arm state and renders the endoscope view.

The entry point is `openxr_pybullet.launch.py
<https://github.com/adeguet1/sawOpenXR/blob/main/ros/launch/openxr_pybullet.launch.py>`_
in ``saw_openxr``. Its profile is in ``sawOpenXR/share/dvrk_pybullet``.

Prepare and launch
##################

Build and source a workspace containing ``saw_openxr``, ``dvrk_robot``,
``dvrk_console``, ``dvrk_simulator_base``, and ``dvrk_pybullet``. Prepare the
PyBullet interpreter as described under :ref:`dvrk_pybullet`. The supplied
OpenXR runtime YAML selects the ``egl`` renderer, so this profile needs a
working EGL configuration for its camera.

Connect the headset to a working OpenXR runtime before starting the session.
The `sawOpenXR setup instructions
<https://github.com/adeguet1/sawOpenXR#readme>`_ describe WiVRn for Quest and
other standalone headsets, including the USB connection option.

From the workspace root:

.. code-block:: bash

   source install/setup.bash
   ros2 launch saw_openxr openxr_pybullet.launch.py \
     exercise:=tray_cubes.yaml rqt:=true

The launch starts the PyBullet ROS frontend and simulator worker,
``dvrk_robot dvrk_system`` with the OpenXR plugin, and
``dvrk_console stereo_display`` for the overlay and desktop preview.
``rqt:=true`` adds arm, console, and diagnostics monitoring.

This profile does not start a control panel or request homing/teleoperation
automatically. Use the system GUI, or open a control panel in a second sourced
terminal:

.. code-block:: bash

   source install/setup.bash
   ros2 run dvrk_console control_panel

Home the simulated system and enable teleoperation with the GUI. MTMR can
control PSM1; MTML can control PSM2 or PSM3. The configuration also supplies
ECM teleoperation mappings.

IO, teleoperation, and video
############################

.. figure:: /images/usage/openxr-pybullet.svg
   :width: 100%
   :align: center
   :alt: OpenXR controller poses and console events enter sawOpenXR and dVRK teleoperation. ROS adapters communicate with PyBullet. Camera frames pass through the console overlay and return to the headset.

   The OpenXR input and video paths meet in ``sawOpenXR``; robot state and
   commands cross the simulator's ROS interface.

**IO:** the OpenXR runtime supplies tracked poses, trigger values, and
button/thumbstick actions. ``mtsOpenXR`` exposes these as ``MTM_GENERIC``
arm interfaces and console events inside ``dvrk_system``. This profile has no
physical MTM or patient-cart IO/PID components.

**Teleoperation:** the console and PSM/ECM teleoperation components connect
OpenXR's cisst interfaces to ``dvrk_psm_from_ros`` and
``dvrk_arm_from_ros`` adapters. These send CRTK commands to PyBullet's ROS
frontend and receive arm state. The frontend communicates with the simulator
worker through private Unix-socket IPC. The OpenXR teleoperation settings use
non-haptic masters and disable physical-MTM alignment.

**Video:** the supplied cart scene publishes stereo to
``@dvrk:simulator:stereo_source``. ``stereo_display`` adds console status,
shows a side-by-side desktop preview, and publishes
``@dvrk:console:stereo_overlay``. ``sawOpenXR`` reads this second Unix-FD
stream and displays it in the headset. The video frames do not travel through
the ROS arm topics or the simulator command IPC.

Controller inputs
#################

The current ``sawOpenXR`` bindings include:

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Input
     - Effect
   * - Push a thumbstick up/away at least 75 percent
     - Release that controller's PSM clutch while held; returning it re-engages the clutch.
   * - Index trigger
     - Control the paired PSM jaw.
   * - Pull both thumbsticks down/toward you
     - Press the console camera input; releasing either releases it.
   * - Hold right A and move the right controller
     - Reposition the video plane while the global clutch is pressed.
   * - Left X
     - Reset the video plane in front of the headset.

``operator_present`` is currently asserted continuously by ``sawOpenXR``;
it is not a headset-worn sensor. The controller clutches and tracking state
must be understood independently. See the `component implementation
<https://github.com/adeguet1/sawOpenXR/blob/main/core/components/code/mtsOpenXR.cpp>`_
and package README for the full input behavior.

Configuration files
###################

The `OpenXR PyBullet profile
<https://github.com/adeguet1/sawOpenXR/tree/main/share/dvrk_pybullet>`_ contains:

.. list-table::
   :header-rows: 1
   :widths: 44 56

   * - File
     - Role
   * - ``system-OpenXR-PyBullet.json``
     - Loads ``mtsOpenXR`` and ROS arm adapters; connects generic arms, console inputs, and teleoperation.
   * - ``pybullet.yaml``
     - Runtime, EGL renderer, rates, and grasp policy.
   * - ``dvrk-console-overlay.json``
     - Reads the simulator stereo socket, adds status, and publishes the OpenXR overlay socket.
   * - ``sawOpenXR-pybullet-unixfd.json``
     - Selects side-by-side video from the overlay socket for the headset.

The launch adds the shared ``ECM_PSM1_PSM2_PSM3.yaml`` cart scene to the
selected ``exercise``. Use ``exercise:=peg_board_ring.yaml`` for a different
exercise or ``headless:=false`` to open the PyBullet desktop viewer too.
