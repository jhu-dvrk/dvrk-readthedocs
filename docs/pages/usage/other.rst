.. _usage-other:

*****
Other
*****

The following examples combine alternative master devices with either a
simulated patient cart or physical patient-side arms. The master-device
components run inside ``dvrk_system``; their console inputs and video paths
can be chosen independently.

.. _usage-force-dimension-isaac:

Force Dimension with Isaac Sim and joystick pedals
##################################################

Force Dimension haptic devices provide the generic MTML and MTMR interfaces
through ``sawForceDimensionSDK``. USB foot pedals configured as a Linux
joystick provide operator presence, clutch, and camera events through
``sawJoystick``. Isaac Sim supplies the patient cart and rendered endoscope
images; no physical patient-cart controllers are required.

.. figure:: /images/usage/force-dimension-isaac-sim.svg
   :width: 100%
   :align: center
   :alt: Force Dimension masters and USB joystick pedals connect to generic master interfaces and the dVRK console. Teleoperation reaches Isaac Sim through ROS arm adapters and private simulator IPC. RTSP stereo video reaches stereo_display, whose overlay socket can be previewed in the control panel.

   Haptic masters, joystick console inputs, and the Isaac Sim patient cart
   use separate device, control, and video paths.

Configuration pieces
********************

This is an integration example assembled from the existing component
configurations, rather than a dedicated JHU launch profile. Prepare Isaac Sim
as described under :ref:`dvrk_isaac_sim`, then combine these pieces in the
system configuration:

* Use ``sawForceDimensionSDK`` with class ``mtsForceDimension`` to expose
  ``MTML`` and ``MTMR`` as ``MTM_GENERIC`` arms. The existing
  ``sawForceDimensionSDK-MTML.json`` and
  ``sawForceDimensionSDK-MTMR.json`` examples show the device names and serial
  fields. Choose the frame transforms and gripper settings for the devices
  being used; :ref:`components-alternative-hardware` describes the supported
  master features.
* Load ``sawJoystick`` with class ``mtsJoystick`` and map its button events
  to the console's ``operator_present``, ``clutch``, and ``camera`` inputs.
  Use ``input_type: COMPONENTS``. The current
  ``jhu-dVRK/system-MTML-PSM1-joystick-console-input.json`` example and
  ``misc/sawJoystickConfiguration.json`` show these component/interface
  mappings. Button indices and the ``/dev/input/js*`` device depend on the
  pedals; see :ref:`config-pedals-usb`.
* Retain the patient-arm ROS adapters from
  ``dvrk_isaac_sim/share/dvrk_systems/system-MTML-MTMR-Haply-patient-cart-ROS.json``:
  ``dvrk_psm_from_ros`` for PSMs and ``dvrk_arm_from_ros`` for ECM. Replace
  that file's Haply masters and simulated console inputs with the two
  components above. Pair MTMR with PSM1, MTML with PSM2, and both masters
  with ECM; a single-device setup can retain just one PSM pairing.

Start the simulator in a sourced terminal:

.. code-block:: bash

   ros2 launch dvrk_isaac_sim simulator.launch.py \
     scene:=ECM_PSM1_PSM2_PSM3_stereo_rtsp.yaml headless:=true

Run ``dvrk_robot dvrk_system`` with the assembled system JSON in a second
terminal. Console events travel directly between cisst components; only
the patient-arm adapter connections use ROS. The Isaac frontend exchanges
commands and state with its worker over private Unix-socket IPC.

Video
*****

Isaac Sim currently publishes native RTSP stereo, normally at
``rtsp://localhost:8554/ECM``. It does not publish the Newton/PyBullet
Unix-FD camera socket. The ``jhu-daVinci-Si/isaac_sim/stereo_display_simulator.json``
example supplies a GStreamer RTSP receive/decode pipeline for
``dvrk_console stereo_display``. Match its decoder and display sink to the
machine, and configure a ``gst_output`` socket if the control panel should
preview the overlay. The control panel consumes that socket; it does not
directly consume the RTSP source. See :ref:`dvrk-stereo-display` for the
display configuration.

.. _usage-haply-real:

Two Haply masters with the JHU da Vinci Si patient cart
#######################################################

This example replaces both MTMs with Haply haptic devices while retaining
the real Si PSMs, ECM, and SUJ. Original foot pedals provide clutch and camera
events. The Goovis HMD displays stereo video and its USB HID head sensor
supplies operator presence. The
ready-to-launch profile is ``dvrk_config_jhu/jhu-daVinci-Si/haply``.

.. figure:: /images/usage/haply-real-patient-cart.svg
   :width: 100%
   :align: center
   :alt: Two Haply devices connect through Haply Service and sawHaplyService to dVRK teleoperation. Original pedals enter through controller IO and the Goovis HMD supplies operator presence over USB. Teleoperation controls the real Si patient cart. Stereo video reaches the Goovis HMD and can be previewed in the control panel.

   Haply replaces the master arms, original pedals retain clutch and camera
   inputs, and the Goovis HMD supplies stereo viewing and operator presence.

Prepare and launch
******************

Build ``sawHaplyService`` in the workspace and start Haply Service. The
profile uses the existing JHU device assignments: MTMR base ``0699`` with
stylus ``1605``, and MTML base ``069E`` with stylus ``1606``. The service
endpoint is ``ws://localhost:10001``. Complete the
:ref:`real-robot startup procedure <usage-real>` for the Si patient cart.

After adding the profile, install the updated configuration package and launch:

.. code-block:: bash

   colcon build --symlink-install --packages-select dvrk_config_jhu
   source install/setup.bash
   ros2 launch dvrk_config_jhu haply_davinci_si.launch.py

The launch starts ``dvrk_robot::dvrk_system``, the SDI stereo pipeline, and
``dvrk_console::control_panel``. Homing and teleoperation remain operator
actions. Use ``video:=false`` to test without starting capture, or
``control_panel:=false`` if another control panel is already running.

The default uses ``PEDALS_GOOVIS_HEAD_SENSOR`` with ``hid/goovis-hd.json``
for the Goovis USB head sensor. Original pedals use the Classic MTML QLA
controller, as named in the JHU configuration.
Keep this controller connected even though it no longer controls an MTM arm.
The Si patient controllers do not supply these pedal inputs.
To use the original ISI head sensor instead of the Goovis head sensor:

.. code-block:: bash

   ros2 launch dvrk_config_jhu haply_davinci_si.launch.py head_sensor:=isi

IO, teleoperation, and video
****************************

``sawHaplyService`` loads ``mtsHaply`` in ``dvrk_system`` and exchanges device
state and commands with Haply Service over WebSockets. It exposes MTML and
MTMR as generic haptic masters. Teleoperation connects directly to the real
patient-arm components, without simulator ROS adapters. MTMR selects PSM1
or PSM3; MTML controls PSM2. The camera pedal selects ECM teleoperation with
both masters. Automatic master alignment is disabled and PSM scale starts
at ``0.2``.

The patient cart keeps the existing JHU Si serials and SUJ-derived base frames:
ECM ``438610``, PSM1 ``334809``, PSM2 ``586288``, and PSM3 ``698534``.
The master HRSV transforms come from the existing two-Haply Isaac profile;
check that the devices' physical placement matches those frames during testing.
This profile has been checked as configuration, not exercised on the hardware.

Video uses the neighboring ``sdi`` profile's DeckLink inputs and alignment
settings. ``stereo_display`` opens one side-by-side window for the Goovis HMD,
retains the Si profile's -150 px horizontal offset, and publishes the
composited ``@dvrk:stereo_display:haply-overlay`` socket for the control panel.
The panel can also preview either eye from the raw and aligned stereo sockets.

The ``haply`` directory contains both head-sensor system JSONs, the two-device
Haply configuration, and the three video pipeline JSONs. No manual assembly
of configuration fragments is required for this JHU profile.

.. _usage-openxr-real:

OpenXR with a real patient cart
###############################

The JHU configurations also connect an OpenXR headset and controllers to
physical patient-cart arms. ``sawOpenXR`` supplies the generic masters and
console input events; the patient-side arms use their real IO and PID
components in ``dvrk_system``. Endoscope video is captured and sent to the
headset with the console overlay.

These examples are in ``dvrk_config_jhu`` under ``jhu-daVinci/open-xr``
and ``jhu-daVinci-Si/open-xr``. They contain site-specific hardware serial
numbers, calibration paths, IO settings, and DeckLink capture devices.
Complete the :ref:`real-robot setup and startup checks <usage-real>` and
match these settings to your hardware. The `sawOpenXR README
<https://github.com/adeguet1/sawOpenXR#readme>`_ also records the package's
hardware-use limitations; these configurations describe an integration and
do not establish that it has been validated for another system.

Launch a JHU profile
********************

With the workspace built and sourced, connect the OpenXR runtime and headset
as in the :ref:`PyBullet example <usage-openxr-pybullet>`. Choose the launch
matching the physical cart.

For the Classic patient cart:

.. code-block:: bash

   source install/setup.bash
   ros2 launch dvrk_config_jhu openxr_davinci.launch.py

For the Si patient cart:

.. code-block:: bash

   source install/setup.bash
   ros2 launch dvrk_config_jhu openxr_davinci_si.launch.py

Each launch starts ``dvrk_robot dvrk_system`` with the cart's OpenXR system
JSON and includes ``dvrk_console stereo_video_pipeline.launch.py`` with the
corresponding ``open-xr`` directory as ``config_dir``. That pipeline starts
``dvrk_data stereo_source``, ``dvrk_data stereo_alignment``, and
``dvrk_console stereo_display``. The ``video`` launch argument defaults to
``true``; ``video:=false`` leaves video startup to a separately managed
pipeline. Only one pipeline should own the configured capture devices and
video sockets.

Neither launch starts a simulator, control panel, or automatic homing helper.
Use the system GUI, or start ``ros2 run dvrk_console control_panel`` in a
second sourced terminal. Follow the real-robot power and homing procedure
before enabling teleoperation.

IO, teleoperation, and video
****************************

.. figure:: /images/usage/openxr-real-patient-cart.svg
   :width: 100%
   :align: center
   :alt: OpenXR inputs drive dVRK teleoperation, patient-arm PID control, and real controller IO. Endoscope SDI video passes through DeckLink capture, stereo alignment, and console overlay to sawOpenXR and the headset.

   The real cart uses hardware IO and calibrated arm control. Its video path
   starts at the physical endoscope cameras.

**IO:** Classic uses ``fw`` and Si uses ``udpfw`` in the supplied system
JSON. ``sawRobotIO1394``, ``mtsPID``, and the dVRK arm components control the
physical ECM/PSMs. The SUJ component supplies patient-arm base frames.
Physical MTMs and the original console pedals/head sensor are replaced in
these profiles by the OpenXR component's interfaces and events.

**Teleoperation:** the same dVRK console and teleoperation components connect
the generic OpenXR masters directly to the hardware arm interfaces. The
patient side does not use ``dvrk_arm_from_ros`` simulator adapters. ROS still
provides status and operator-control topics for the display and control panel.
The OpenXR settings disable haptic-master behavior and physical-MTM alignment.

.. list-table::
   :header-rows: 1
   :widths: 24 24 24 28

   * - Profile
     - MTMR pairs with
     - MTML pairs with
     - Camera teleoperation
   * - Classic
     - PSM1
     - PSM2
     - MTML and MTMR to ECM
   * - Si
     - PSM1 or PSM3
     - PSM2
     - MTML and MTMR to ECM

The per-hand clutch event follows the selected hand's PSM pairing. As in the
simulation example, ``sawOpenXR`` currently asserts ``operator_present``
continuously; this is not the original physical head-sensor behavior.

**Video:** the JHU files capture left and right SDI inputs from DeckLink
devices 0 and 1. ``stereo_source`` packs them into the side-by-side
``@dvrk:stereo_source:stereo`` socket. ``stereo_alignment`` consumes that
stereo stream, aligns its eyes, and publishes
``@dvrk:stereo_alignment:stereo``. ``stereo_display`` uses its ``headless``
sink to publish composited video at
``@dvrk:stereo_display:openxr-overlay``. ``sawOpenXR`` receives that socket
and renders the stereo view in the headset. See :ref:`Stereo display
<dvrk-stereo-display>` for alignment, overlay, and display configuration.

Configuration files
*******************

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - File in the profile's ``open-xr`` directory
     - Role
   * - ``system-OpenXR-daVinci.json`` (Classic) or ``system-OpenXR-daVinciSi.json`` (Si)
     - Real arm serials, SUJ/base frames, IO, generic masters, console events, and teleoperation mappings.
   * - ``stereo_source.json``
     - Physical left/right capture devices and the packed stereo source socket.
   * - ``stereo_alignment.json``
     - Eye alignment, crop, color settings, and aligned stereo socket.
   * - ``stereo_display.json``
     - Console overlay, headless output, and display offset.
   * - ``sawOpenXR-dvrk-socket.json``
     - Receives the composited stereo socket for the headset.

Sources: `Classic launch
<https://github.com/jhu-dvrk/dvrk_config_jhu/blob/main/launch/openxr_davinci.launch.py>`_,
`Classic configuration
<https://github.com/jhu-dvrk/dvrk_config_jhu/tree/main/jhu-daVinci/open-xr>`_,
`Si launch
<https://github.com/jhu-dvrk/dvrk_config_jhu/blob/main/launch/openxr_davinci_si.launch.py>`_,
and `Si configuration
<https://github.com/jhu-dvrk/dvrk_config_jhu/tree/main/jhu-daVinci-Si/open-xr>`_.
