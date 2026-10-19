.. _usage-other:

*****
Other
*****

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
devices 0 and 1. ``stereo_source`` publishes the eye streams to
``@dvrk:stereo_source:left`` and ``@dvrk:stereo_source:right``.
``stereo_alignment`` aligns them and publishes
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
     - Physical left/right capture devices and source sockets.
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
