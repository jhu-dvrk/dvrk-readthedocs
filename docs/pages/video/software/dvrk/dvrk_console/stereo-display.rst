.. _dvrk-stereo-display:

Stereo display and calibration
##############################

``stereo_display`` is the low-latency rendering endpoint for the HRSV, Goovis,
and other surgeon-console stereo displays.  It consumes an already aligned
side-by-side stream, adds optional video and dVRK status overlays, and renders
through GStreamer OpenGL sinks.

Run it with a :doc:`stereo-display-configuration`:

.. code-block:: bash

   ros2 run dvrk_console stereo_display -c stereo_display.json

Options
*******

``--grid`` displays the calibration grid.  ``-g 0`` through ``-g 3`` writes a
GStreamer Graphviz pipeline at increasing detail when
``GST_DEBUG_DUMP_DOT_DIR`` is set.

``gtkglsink`` requires GLX.  Unless ``GDK_BACKEND`` is already set, the
application selects ``x11`` so it can run through XWayland in a Wayland
session.

Rendering and outputs
*********************

``glimage`` opens one side-by-side window.  ``glimages`` opens one window per
eye.  The control window reports sink frame rates, toggles the status overlay,
and adjusts the extra-stream scale when those streams are configured.

The HUD subscribes to the configured dVRK console namespace and presents
operator presence, camera and clutch pedals, focus/coag inputs, selected
teleoperations, following state, instrument names, invalid PSM pose status,
and ECM teleoperation.  These are small ROS messages; the video itself remains
outside ROS.

Optional ``unixfdsinks`` expose the unannotated ``stereo`` stream or rendered
``overlay`` stream for recording, :doc:`control-panel`, or other local
consumers.  See :doc:`/pages/video/software/dvrk/dvrk_data/timestamps` for the
additional stereo and overlay output observations attached to these buffers.

Extra and AR streams
********************

Up to two additional mono or stereo sources can be shown as a
picture-in-picture strip.  Mono streams are duplicated for both eyes; stereo
entries provide a source for each eye.

AR mode receives independent left and right graphic streams from abstract
Unix sockets and blends them over the endoscope with ``glvideomixer``.  An
optional RGB color key makes a selected background transparent.  AR buffer PTS
is rewritten to current pipeline running time so a variable-rate generator can
update the latest overlay without controlling the endoscope frame rate.

Camera alignment versus display calibration
*********************************************

There are two independent calibration steps:

#. :ref:`dvrk-video-data` camera alignment corrects crop, vertical alignment,
   horizontal alignment, and color differences before the stereo stream is
   published.
#. ``stereo_display_calibration`` adjusts
   ``display_horizontal_offset_px`` for fusion at the surgeon's intended
   working depth.

Run the display calibration only after ``stereo_source`` and
``stereo_alignment`` are producing the configured stereo socket:

.. code-block:: bash

   ros2 run dvrk_console stereo_display_calibration -c stereo_display.json

The script requires ``stereo.socket`` and its per-eye size.  Use left and right
arrows to change the offset, ``g`` to toggle the grid, ``f`` for fullscreen,
and ``q`` to finish and choose whether to save.  Add ``--split`` to preview one
window per eye; it is selected automatically for a ``glimages`` configuration.

Launching the complete video path
*********************************

``da_vinci.launch.py`` starts the standard three-process video pipeline in
dependency order:

.. code-block:: text

   0 s  dvrk_data/stereo_source
   2 s  dvrk_data/stereo_alignment
   4 s  dvrk_console/stereo_display

With all files in one directory:

.. code-block:: bash

   ros2 launch dvrk_console da_vinci.launch.py \
     config_dir:=/path/to/video/config

The default names are ``stereo_source.json``, ``stereo_alignment.json``, and
``stereo_display.json``.  Override any with an absolute path or a filename
relative to the selected base directory.

A system-directory layout can also be selected:

.. code-block:: bash

   ros2 launch dvrk_console da_vinci.launch.py \
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

.. important::

   Despite its filename, ``da_vinci.launch.py`` starts only the video source,
   alignment, and display applications.  It does not start ``dvrk_system``,
   robot arms, or ``control_panel``.
