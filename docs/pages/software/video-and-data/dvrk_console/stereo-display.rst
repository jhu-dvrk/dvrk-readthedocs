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
  Runs the pipeline without creating on-screen display windows. The composited
  stereo stream is published to a local ``@dvrk`` socket or shared-memory buffer.
  This is typically used for headless recording, web streaming, or passing video
  frames to OpenXR spatial computing applications (such as Apple Vision Pro or
  Meta Quest).

Along with the video displays, a **Control Window** is created:

.. figure:: /images/gui/stereo_overlay_gui.png
   :width: 400px
   :align: center

   Stereo display control window

The control window allows the operator to:

* Toggle the status HUD overlay on or off (**"Overlay"**);
* Toggle left and right eye corner labels (**"L/R Labels"**);
* Select the display monitor and toggle fullscreen for each output window;
* Dynamically adjust picture-in-picture stream scale (when extra streams are configured);
* Quit the application (**"Quit"**).

.. note::

   The calibration grid is not in this window; it is enabled via the
   ``--grid`` command-line option on ``stereo_display`` or dynamically in
   ``stereo_display_calibration``. Overlay opacity is configured via
   ``"overlay_alpha"`` in ``stereo_display.json``.


The Cairo Head-Up Display (HUD) engine
**************************************

To provide critical robotic feedback without diverting the surgeon's gaze,
``stereo_display`` includes a high-performance vector Head-Up Display (HUD).
The HUD is rendered using **Cairo** directly onto the OpenGL frame textures
inside the GStreamer display sink:

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

* Video playback and rendering are never blocked by ROS 2 middleware processing;
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
2. **Hardware Compositing**: Graphics are blended with the camera frames using
   GStreamer's ``glvideomixer`` element on the GPU.
3. **Chroma Key Transparency**: An optional RGB color key (e.g. ``[0, 255, 0]``
   green) treats that color as fully transparent, allowing standard 2D/3D
   graphics applications to overlay shapes without needing an explicit alpha
   channel.
4. **Asynchronous Frame Timing (PTS Rewriting)**: AI segmentation and detection
   models often run at lower or variable rates (e.g. 15–30 Hz) compared to the
   60 Hz endoscopic video. The AR pipeline rewrites buffer Presentation Time
   Stamps (PTS) to the running pipeline clock, ensuring that the latest available
   AI annotation displays without stalling the 60 Hz live video feed.


Camera alignment versus display calibration
*******************************************

Two distinct calibration steps must be performed:

1. **Camera Alignment (``stereo_alignment``)**:
   Calibrates the physical cameras. It corrects optical differences between the
   left and right sensors, including vertical disparities, horizontal shifts,
   optical roll/rotation, and color balance before publishing
   ``@dvrk:stereo_alignment:stereo``. See :ref:`dvrk-video-data`.
2. **Display Calibration (``stereo_display_calibration``)**:
   Calibrates the surgeon display. It adjusts the horizontal disparity
   (``display_horizontal_offset_px``) so that the stereo convergence plane
   matches the surgeon's eyes and working distance at the console.

To calibrate the display offset:

.. code-block:: bash

   ros2 run dvrk_console stereo_display_calibration -c stereo_display.json

* Use the **Left** and **Right** arrow keys to adjust the pixel offset until stereo
  fusion is comfortable.
* Press **g** to toggle the alignment grid.
* Press **f** to toggle fullscreen mode.
* Press **q** to quit and save the new offset to ``stereo_display.json``.

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
