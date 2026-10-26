.. _dvrk-stereo-display:

Stereo display and calibration
##############################

``stereo_display`` is the low-latency rendering endpoint for the HRSV, Goovis,
and other surgeon-console stereo displays.  It consumes an already aligned
side-by-side stream, adds optional video and dVRK status overlays, and renders
through GStreamer OpenGL sinks.

Run it with a :ref:`dvrk-stereo-display-configuration`:

.. code-block:: bash

   ros2 run dvrk_console stereo_display -c stereo_display.json

Options
*******

``--grid`` displays the calibration grid.

For a graph of the negotiated display pipeline, see
:ref:`dvrk-gstreamer-dot-files`.

``gtkglsink`` requires GLX.  Unless ``GDK_BACKEND`` is already set, the
application selects ``x11`` so it can run through XWayland in a Wayland
session.

Rendering and display topologies
********************************

``stereo_display`` supports three primary sink topologies configured in the
``sinks`` array:

``separate`` (Classic HRSV)
  Opens two independent fullscreen windows, one for the left eye and one for the
  right eye. This topology is designed for the classic da Vinci High-Resolution
  Stereo Viewer (HRSV), which uses two separate video cables (DVI/HDMI) driven
  by a dedicated graphics card.

``side_by_side`` (3D Monitors and Headsets)
  Opens a single window displaying the left and right eye images packed horizontally
  side by side. This mode is used for 3D TVs, passive polarizing 3D monitors, and
  external 3D video headsets such as Gooviz.

``headless`` (Socket Streaming and XR)
  Creates no video-output windows; the control window is still available.
  The main stereo stream and HUD can be published through the configured
  local ``@dvrk`` output socket for recording or XR consumers. The default
  socket is ``@dvrk:stereo_display:overlay``; an empty ``gst_output`` disables it.

Along with the video displays, a **Control Window** is created:

.. figure:: /images/gui/stereo_overlay_gui.png
   :width: 400px
   :align: center

   Stereo display controls (earlier version, before the Frame timing panel)

The control window allows the operator to:

* Toggle the status HUD overlay on or off (**"Overlay"**);
* Toggle left and right eye corner labels (**"L/R Labels"**);
* Select the display monitor and toggle fullscreen for each output window;
* Inspect per-eye frame age, channel time difference, source FPS, and stale
  indicators in **Frame timing**;
* Clear collected timing history with **Reset statistics**;
* Dynamically adjust picture-in-picture stream scale (when extra streams are configured);
* Quit the application (**"Quit"**).

.. note::

   The calibration grid is not in this window; it is enabled via the
   ``--grid`` command-line option on ``stereo_display`` or dynamically in
   ``stereo_display_calibration``. Overlay opacity is configured via
   ``"overlay_alpha"`` in ``stereo_display.json``.


.. _dvrk-frame-timing:

Frame timing panel
******************

The control window always shows **Frame timing**; it requires no JSON option.
Statistics refresh four times per second. Ages and channel differences are
shown in milliseconds.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Value
     - Meaning
   * - Left / Right: Current
     - Age of the most recently received source image for that eye. It keeps
       increasing when buffers stop arriving.
   * - Left / Right: Avg, Max
     - Average and maximum ages sampled when buffers reached the display
       input during the last five seconds. Reused source images still
       contribute their actual age to these statistics.
   * - Left / Right: Peak
     - Largest age observed since the statistics were reset, including
       the growing age of a stalled frame.
   * - Left / Right: FPS
     - Rate of source-timestamp changes over the last five seconds, or the
       shorter interval since startup/reset. Repeated buffers carrying the
       same source timestamp do not count as new camera images. This is
       independent of stereo-buffer rate and monitor refresh rate.
   * - Δt: Current
     - Signed difference ``left source time - right source time`` for the
       images most recently received at the display. A negative value means
       the left image's source observation is older.
   * - Δt: Avg, Max
     - Average and maximum absolute channel differences in the last five
       seconds, so positive and negative offsets do not cancel.

An eye is marked **STALE** when its source timestamp has not changed at the
display for 500 ms, even if repeated buffers continue arriving. If no samples
remain in the rolling window, Avg and Max show ``--`` while Current and Peak
can continue aging. Missing source timestamps, or source times later than the
observing CPU's realtime clock, also show ``--`` rather than a zero age.
**Reset statistics** clears the rolling history and peaks while video keeps
running.

Ages use the CPU source observations carried with the selected pixels and are
measured at the display sink input. In separate-eye mode each sink is measured
independently. They exclude latency before the source probe and the physical
monitor's scanout. Source observations use ``CLOCK_REALTIME``; elapsed aging
between received buffers uses a steady clock. The panel does not compare
independent hardware-clock epochs. See :ref:`dvrk-timestamps` for clock domains
and :ref:`dvrk-latency-calibration` for an empirical camera latency estimate.

For diagnosis:

* An increasing per-eye age with little or no source FPS indicates reused or
  stalled camera content, even if overlays or buffer delivery continue.
* A large absolute Δt indicates separation between the selected eye images.
  The pipeline reports it without delaying one eye to match the other.
* Growing ages on both eyes with a modest Δt indicate a common delay in the
  upstream path; compare source, alignment, and display stages.

Latest-frame display layouts
****************************

Display shifts, PiP, and AR use the same :ref:`dvrk-latest-frame-policy` as
source and alignment. Each spatial mixer caches one image per input and
composes the latest available images when an input changes. A slow auxiliary
stream does not gate camera updates. Input PTS are not matched between eyes;
original source observations remain available for the timing panel.

Layout scaling and blending operate on CPU frames. The final video windows
upload the completed images through ``glupload`` and ``gtkglsink`` with sink
clock synchronization disabled. The output can update at a different rate
from either eye's distinct-source FPS.

The Cairo Head-Up Display (HUD) engine
**************************************

To provide critical robotic feedback without diverting the surgeon's gaze,
``stereo_display`` includes a high-performance vector Head-Up Display (HUD).
The HUD is drawn with **Cairo** on CPU video frames before the final upload
to the OpenGL display sinks:

.. figure:: /images/gui/stereo_overlay_side_by_side_gooviz.png
   :width: 400px
   :align: center

   Side-by-side stereo display with status overlay (Goovis format)

Event loop and thread integration
=================================

The overlay engine uses a specialized GLib/ROS executor (``glib_ros_executor``)
that integrates ROS 2 subscriber callbacks into the GTK/GLib main event loop.
State updates are protected by a mutex and cached in an internal ``OverlayState``
structure. When a video frame arrives, Cairo draws the latest cached state
vectorially, ensuring that:

* Streaming threads use cached state without waiting for a new ROS 2 message;
* Overlays scale cleanly to any resolution (from SD up to 4K);
* Text, icons, and status badges remain crisp and anti-aliased.

HUD components and telemetry
============================

The HUD subscribes to CRTK topics under the configured console namespace
(default ``/console`` or ``/dvrk``) and displays:

* **Pedal and Input States**: Real-time indicators for the clutch pedal, camera
  pedal, operator head-presence sensor, coag/bicoag inputs, and camera focus
  buttons.
* **Teleoperation Arm Bindings**: Shows which Patient Side Manipulator (e.g.
  PSM1, PSM2, PSM3) is currently paired with each Master Tool Manipulator (MTML,
  MTMR).
* **Following State & Motion Scale**: Displays whether teleoperation following
  is active, paused, or clutching, along with the current motion scaling ratio
  (e.g., ``1:1``, ``1:2``, ``1:3``).
* **Instrument Identification**: Queries the active tool type (e.g., Large
  Needle Driver, ProGrasp, Cautery Hook) for each arm.
* **Horizon & Gravity Indicator**: Dynamic roll angle and gravity vector indicator
  reflecting the endoscopic camera's orientation in space.
* **Safety Warnings**: Instantaneous warning banners if an arm reaches joint
  limits, safety stops, or invalid Cartesian position states (``measured_cp``).
* **Eye Labels & Alignment Grid**: Left/right corner badges (toggled via the
  **"L/R Labels"** control button) and an optical alignment grid (enabled via
  the ``--grid`` command-line option).


Picture-in-picture (PiP) and augmented reality (AR)
***************************************************

Picture-in-Picture (PiP)
========================

Up to two additional mono or stereo video feeds can be rendered as a
picture-in-picture strip over the live surgical feed.

* **Mono feeds** (e.g., external laparoscopic overview or ultrasound) are
  automatically duplicated to both eyes.
* **Stereo feeds** provide dedicated left and right views.
* Integration with **3D Slicer**: External medical imaging or ultrasound can be
  streamed directly into the surgeon console using the `SlicerGStreamer
  <https://github.com/rosmed/SlicerGStreamer>`_ plugin via local Unix sockets.

Augmented Reality (AR) blending
===============================

AR mode allows external computer vision algorithms, AI inference models, or
surgical planning systems to draw graphics directly on top of the live endoscope:

1. **Independent Stereo AR Streams**: Left and right AR graphics streams are
   received from abstract Unix sockets (e.g., ``@dvrk:stereo_source:left_ar``).
2. **Latest-image Compositing**: Arrival-driven spatial mixers blend the
   newest available graphics and camera images on CPU frames. The result is
   uploaded to the OpenGL display sinks.
3. **Chroma Key Transparency**: An optional RGB color key (e.g. ``[0, 255, 0]``
   green) treats that color as fully transparent, allowing standard 2D/3D
   graphics applications to overlay shapes without needing an explicit alpha
   channel.
4. **Independent Input Timing**: Graphics and camera inputs can update at
   different rates. Each mixer uses its newest cached images without matching
   input PTS or waiting for a peer update. New output PTS describe publication;
   the camera's original source observations remain attached for diagnostics.
   Optional AR inputs supply transparent images when unavailable, and stale AR
   content is cleared after its input timeout. See :ref:`dvrk-control-panel`.


Camera alignment versus display calibration
*******************************************

Calibrate the camera pair first with the ``dvrk_data`` launch described in
:ref:`dvrk-video-data`. That step saves the crop, horizontal and vertical
alignment, and color balance in ``stereo_alignment.json``. At runtime,
``stereo_alignment`` applies those values and publishes the aligned stereo
socket used by the display.

Next, from the directory containing ``stereo_source.json``,
``stereo_alignment.json``, and ``stereo_display.json``, run:

.. code-block:: bash

   ros2 launch dvrk_console stereo_display_calibration.launch.py

This starts source, alignment, and the GTK display calibration tool. The tool
can show one side-by-side preview or two movable eye windows; **Fullscreen
previews** fills the window height. **Left/Right** adjusts the video shift and
**Up/Down** adjusts widget disparity, one pixel per key, while either the
control window or a preview has focus. The HUD icons show the selected widget
depth relative to the shifted video. **Save & quit** writes the values to
``stereo_display.json`` and stops the launched video processes; **Quit** asks
about unsaved changes.

``display_horizontal_shift_px`` changes the video convergence for this display.
``widget_horizontal_disparity_px`` positions fixed HUD widgets and extra-stream
tiles at a comfortable depth. Both are measured in final per-eye pixels;
positive disparity moves left-eye pixels right and right-eye pixels left. The
legacy ``display_horizontal_offset_px`` affected widgets, not video, and is
converted to widget disparity with the opposite sign when saved.

Use ``config_dir:=/path/to/configs`` for another directory, or override
``source_config``, ``alignment_config``, and ``display_config`` individually.

Launching the complete video path
*********************************

``stereo_video_pipeline.launch.py`` starts the standard three-process stereo
video pipeline in dependency order:

.. code-block:: text

   0 s  dvrk_data/stereo_source
   2 s  dvrk_data/stereo_alignment
   4 s  dvrk_console/stereo_display

With all files in one directory:

.. code-block:: bash

   ros2 launch dvrk_console stereo_video_pipeline.launch.py \
     config_dir:=/path/to/video/config

The default names are ``stereo_source.json``, ``stereo_alignment.json``, and
``stereo_display.json``.  Override any with an absolute path or a filename
relative to the selected base directory.

A system-directory layout can also be selected:

.. code-block:: bash

   ros2 launch dvrk_console stereo_video_pipeline.launch.py \
     config_parent:=/path/to/systems \
     system:=jhu-daVinci

This uses ``/path/to/systems/jhu-daVinci`` as the base directory.
``config_parent`` is required whenever ``system`` is non-empty.

.. list-table::
   :header-rows: 1

   * - Argument
     - Default
     - Meaning
   * - ``config_dir``
     - current directory
     - Base directory when ``system`` is empty.
   * - ``system``
     - empty
     - Directory name below ``config_parent``.
   * - ``config_parent``
     - empty
     - Parent of system configuration directories.
   * - ``source_config``
     - ``stereo_source.json``
     - Source filename or absolute path.
   * - ``alignment_config``
     - ``stereo_alignment.json``
     - Alignment filename or absolute path.
   * - ``display_config``
     - ``stereo_display.json``
     - Display filename or absolute path.

All three paths are checked before starting a process.  The fixed delays aid
startup ordering but are not readiness checks; inspect active sockets with
``ros2 run dvrk_data gscam_socket`` when troubleshooting.
