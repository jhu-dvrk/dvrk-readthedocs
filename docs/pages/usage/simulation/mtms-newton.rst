.. _usage-mtms-newton:

******************
MTMs with Newton
******************

This example uses the physical JHU Classic MTML and MTMR, console foot
pedals, and head sensor to teleoperate a Newton patient cart. The ECM and
PSM1--PSM3 are simulated. Newton's stereo camera is displayed in the HRSV.

The entry point is `davinci_newton.launch.py
<https://github.com/jhu-dvrk/dvrk_config_jhu/blob/main/launch/davinci_newton.launch.py>`_
in ``dvrk_config_jhu``. It selects the ``jhu-daVinci/newton`` profile through
the shared ``jhu_launch`` helper.

Prepare and launch
##################

Build the dVRK workspace, including ``dvrk_robot``, ``dvrk_console``,
``dvrk_config_jhu``, and ``dvrk_newton``, and prepare the Newton environment
as described under :ref:`dvrk_newton`. Connect and power the physical MTM
controllers using the :ref:`real-robot startup procedure <usage-real>`.
The supplied system JSON contains JHU MTM serial numbers and calibration
paths, so adapt these to your console before running it elsewhere.

From the workspace root:

.. code-block:: bash

   source install/setup.bash
   ros2 launch dvrk_config_jhu davinci_newton.launch.py \
     exercise:=tray_cubes.yaml rqt:=true

The launch starts the Newton ROS frontend and its simulator worker,
``dvrk_robot dvrk_system``, ``dvrk_console stereo_display``, and
``dvrk_console control_panel``. ``rqt:=true`` adds arm, console, and diagnostics
monitoring. Use the control panel to home the system and enable teleoperation;
the launch does not request these actions automatically.

The configured pairings are MTMR with PSM1, and MTML with either PSM2 or PSM3.
Both MTMs are configured for ECM teleoperation through the console's camera
input. Pedals and the physical head sensor retain their console roles.

IO, teleoperation, and video
############################

.. figure:: /images/usage/mtms-newton.svg
   :width: 100%
   :align: center
   :alt: Physical MTMs and console inputs connect through FireWire IO and PID control to dVRK teleoperation. ROS adapters connect to Newton; Newton stereo video flows through stereo_display to the HRSV.

   Physical console IO stays in ``dvrk_system``. Newton supplies the patient
   arms and endoscope images.

**IO:** ``sawRobotIO1394`` communicates with the MTM controllers over the
configured ``fw`` port. ``mtsPID`` and the dVRK MTM arm components control the
physical masters. The same IO component supplies the pedal and head-sensor
events to the console.

**Teleoperation:** the console selects the active PSM/ECM teleoperation
components. ``dvrk_psm_from_ros`` and ``dvrk_arm_from_ros`` expose the simulated
PSMs and ECM as cisst arm interfaces. Their ``PSM_GENERIC`` and ``ECM_GENERIC``
entries in the system JSON connect teleoperation to the simulator's ROS 2
CRTK command and state topics. Inside the simulator, the ROS frontend and
Newton worker exchange commands and state over private Unix-socket IPC.

**Video:** the Newton camera publishes side-by-side stereo to
``@dvrk:simulator:stereo_source``. ``dvrk_console stereo_display`` reads that
Unix-FD stream, subscribes to console status for its overlay, and opens
separate left/right display windows for the HRSV. The physical head sensor
supplies an input event; it does not carry video. This session uses rendered
camera images, so it does not start hardware capture or stereo alignment.

Configuration files
###################

The `jhu-daVinci/newton directory
<https://github.com/jhu-dvrk/dvrk_config_jhu/tree/main/jhu-daVinci/newton>`_
contains the profile files selected by the launch:

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - File
     - Role
   * - ``system-daVinci-Newton.json``
     - Physical MTMs and console IO; ROS patient-arm adapters; teleoperation bindings.
   * - ``newton_patient_cart.yaml``
     - Newton runtime, device, rates, and grasp settings.
   * - ``ECM_PSM1_PSM2_PSM3.yaml``
     - Includes ``../simulator_cart.yaml`` and configures the stereo camera and video socket.
   * - ``stereo_display_simulator.json``
     - Reads rendered video; selects separate HRSV windows and the widget disparity.

The supplied camera profile renders 800 by 600 pixels per eye at 15 Hz.
``eye_size: auto`` in the display JSON follows the stream dimensions. Edit
the scene for camera settings and the display JSON for monitor layout or
display convergence.

To use a different exercise, set ``exercise:=peg_board_ring.yaml``. To open
the Newton desktop viewer as well, set ``headless:=false``. For the JHU Si
console's MTMs and Goovis input/display configuration, use
``davinci_si_newton.launch.py`` instead; it selects ``jhu-daVinci-Si/newton``.
