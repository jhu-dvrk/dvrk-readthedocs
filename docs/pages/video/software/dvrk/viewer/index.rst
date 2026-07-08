.. _stereo-viewer:

Surgeon Console Visualization
=============================

.. warning::
   The dVRK surgeon console visualization tools are exclusively supported on ROS 2. ROS 1 is **not** supported.

The ``dvrk_console`` package contains applications intended for the surgeon's
console.  It is focused on visualization and operator interaction, not on
offline data collection.  The main tools are:

* ``stereo_display``: a low-latency stereo display pipeline for the HRSV and
  other stereo displays.  It renders the endoscope feed, optional dVRK status
  overlays, picture-in-picture streams, and AR overlays using native GStreamer
  elements.
* ``control_panel``: a simplified operator-facing control panel.  It is meant
  for common console interactions and status checks, while the main
  ``dvrk_system`` UI remains the more complete debug/engineering interface.

You can launch the viewer using the ``stereo_display`` executable, for example:
``ros2 run dvrk_console stereo_display``.

See a video demonstration of the stereo viewer and its overlay features on the `@dvrk-robot <https://www.youtube.com/@dvrk-robot>`_ YouTube channel: `dVRK Stereo Display Example <https://youtu.be/NyHr2-rO-T0?si=KeEKETHXyHFg2tmx>`_.

Console-Oriented Rendering
--------------------------

Using GStreamer directly provides dramatically lower latency compared to
routing high-definition stereo streams through ROS image topics.  At its core,
``stereo_display`` constructs a native display pipeline for the master console.
The display consumes configured camera streams, composes the stereo image and
overlays, and renders directly through GStreamer sinks.

The broader ROS 2 video architecture still supports modular multi-process
pipelines through ``dvrk_data`` and ``unixfd`` inter-process video routing.
Because ``unixfd`` passes local file descriptors, processes exchanging video in
this way must run on the same computer.

In addition to rendering the video feed, ``stereo_display`` displays icons and
textual overlays that summarize the operational state of the
:ref:`dVRK System Node <system>` within the surgeon's console.  These overlays
subscribe to lightweight ROS 2 status topics, so the main dVRK system node can
run on a separate computer when the video path remains local to the display
computer.

Running the Viewer
------------------

The package provides the ``stereo_display`` executable for 3D displays such as
the HRSV in the surgeon's console. It can be launched via ROS 2:

.. code-block:: bash

   ros2 run dvrk_console stereo_display -c <config.json> [--grid] [-g <0|1|2|3>]

Command-Line Options
~~~~~~~~~~~~~~~~~~~~

* **``-c <config.json>``**: (Required) Path to the JSON configuration file.
* **``--grid``**: (Optional, ``stereo_display`` only) Displays a calibration grid and white circle overlay to help align/fuse the display horizontally.
* **``-g <level>`` / ``--dot <level>``**: (Optional) Dumps the constructed GStreamer pipeline graph as a Graphviz ``.dot`` file. To use this feature, the ``GST_DEBUG_DUMP_DOT_DIR`` environment variable must be set (e.g. ``export GST_DEBUG_DUMP_DOT_DIR=/tmp``).
  
  Available detail levels:
  
  * ``0``: Minimal (elements only)
  * ``1``: Light (elements and states, recommended)
  * ``2``: Medium (adds media types)
  * ``3``: Full (all parameters)

Environment Variables
~~~~~~~~~~~~~~~~~~~~~

* **``GDK_BACKEND``**: The viewer relies on ``gtkglsink`` for GTK-embedded OpenGL hardware rendering, which requires OpenGL via GLX. Under Wayland sessions, the viewer automatically forces ``GDK_BACKEND=x11`` to run via XWayland unless it is already set in the environment. You can manually override this if needed.

.. toctree::

   configuration
   calibration_tool
   pipeline
