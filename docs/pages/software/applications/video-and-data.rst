.. _applications-video-and-data:

##############
Video and data
##############

The ``dvrk_data`` and ``dvrk_console`` packages provide applications for live
video acquisition, low-latency stereo display, operator controls, and
synchronized data collection.

All applications in this section require **ROS 2**.


Live pipeline and display
*************************

The live video pipeline applications look for their configuration files
(``stereo_source.json``, ``stereo_alignment.json``, and ``stereo_display.json``)
in the current working directory by default. When launched from a directory
containing these files, configuration arguments are optional.

.. _app-stereo-video-pipeline:

``stereo_video_pipeline.launch.py``
===================================

* ROS 2 launch file from ``dvrk_console``
* Starts the entire live stereo acquisition, alignment, and display pipeline in a single command

If the current working directory contains the configuration files, simply run:

::

   ros2 launch dvrk_console stereo_video_pipeline.launch.py

You can also specify a custom configuration folder using ``config_dir``:

::

   ros2 launch dvrk_console stereo_video_pipeline.launch.py \
       config_dir:=<path-to-json-configs>

This launch file coordinates the startup sequence of:

1. :ref:`stereo_source <app-stereo-source>`
2. :ref:`stereo_alignment <app-stereo-alignment>` (delayed by 2 seconds)
3. :ref:`stereo_display <app-stereo-display>` (delayed by 4 seconds)

For details on configuration and arguments, see :ref:`dvrk-stereo-display`.


.. _app-control-panel:

``control_panel``
=================

* C++ application with a GTKmm graphical interface from ``dvrk_console``
* Operator control panel for robot power, arm homing, teleoperation activation, clutching, and camera feed preview
* https://github.com/jhu-dvrk/dvrk_console

::

   ros2 run dvrk_console control_panel

For details on available options and features, see :ref:`dvrk-control-panel`.


.. _app-stereo-display:

``stereo_display``
==================

* C++ application with GTKmm and OpenGL/GStreamer from ``dvrk_console``
* Low-latency surgeon display (for HRSV, 3D monitor, or VR headset) with Cairo graphical overlays for robot state, arm clutch, and camera controls
* https://github.com/jhu-dvrk/dvrk_console

.. note::

   This node is typically started automatically by
   :ref:`stereo_video_pipeline.launch.py <app-stereo-video-pipeline>`, rather than
   launched manually.

When run standalone in a directory with ``stereo_display.json``, no argument is
needed:

::

   ros2 run dvrk_console stereo_display

You can also pass an explicit configuration file:

::

   ros2 run dvrk_console stereo_display -c <path-to-stereo-display.json>

For display configuration and supported modes, see :ref:`dvrk-stereo-display` and :ref:`dvrk-stereo-display-configuration`.


.. _app-stereo-source:

``stereo_source``
=================

* C++ application with GStreamer from ``dvrk_data``
* Acquires left and right video streams from capture hardware (V4L2, DeckLink, etc.) and provides high-bandwidth local ``unixfd`` sockets
* https://github.com/jhu-dvrk/dvrk_data

.. note::

   This node is typically started automatically by
   :ref:`stereo_video_pipeline.launch.py <app-stereo-video-pipeline>`.

::

   ros2 run dvrk_data stereo_source -c stereo_source.json

For configuration details, see :ref:`dvrk-video-data` and :ref:`dvrk-video-configuration`.


.. _app-stereo-alignment:

``stereo_alignment``
====================

* C++ application with GStreamer from ``dvrk_data``
* Applies real-time transformation, crop, and rectification matrices to align left and right video streams
* https://github.com/jhu-dvrk/dvrk_data

.. note::

   This node is typically started automatically by
   :ref:`stereo_video_pipeline.launch.py <app-stereo-video-pipeline>`.

::

   ros2 run dvrk_data stereo_alignment -c stereo_alignment.json

For configuration details, see :ref:`dvrk-video-data`.


Data collection, annotation and extraction
******************************************

.. _app-record:

``record``
==========

* C++ application with GStreamer and rosbag2 from ``dvrk_data``
* Synchronized multi-channel recorder for stereo video streams, audio, and robot kinematic telemetry (written to MCAP rosbags and MP4 video files)
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data record -c record.json

For session directory layout and configuration schemas, see :ref:`dvrk-record`.


.. _app-video-tag:

``video_tag``
=============

* C++ application with GTKmm and GStreamer from ``dvrk_data``
* Interactive playback GUI for tagging and annotating event ranges in recorded video sessions
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data video_tag -v <session>/<video>.mp4 -c record.json

For tag schemas and tagging workflows, see :ref:`dvrk-tagging`.


.. _app-extract:

``extract``
===========

* Python CLI utility from ``dvrk_data``
* Batch extraction tool to slice synchronized video clips, audio, and robot telemetry based on tag files or time ranges
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data extract -d <session-directory> --all

For extraction options and formats, see :ref:`dvrk-extract`.


Calibration and utilities
*************************

.. _app-video-latency:

``video_latency``
=================

* C++ application with GStreamer from ``dvrk_data``
* Measures end-to-end glass-to-glass latency of the camera and display pipeline
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data video_latency -c record.json -s <source-name>

For setup and measurement instructions, see :ref:`dvrk-latency-calibration`.


.. _app-stereo-alignment-calibration:

``stereo_alignment_calibration``
================================

* Python utility from ``dvrk_data``
* Interactive tool to calibrate stereo camera offset, scaling, and rotation
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data stereo_alignment_calibration -c stereo_alignment.json

For alignment calibration instructions, see :ref:`dvrk-video-data`.


.. _app-stereo-display-calibration:

``stereo_display_calibration``
==============================

* Python utility from ``dvrk_console``
* Utility to test and configure display outputs, resolutions, and monitor offsets for surgeon consoles
* https://github.com/jhu-dvrk/dvrk_console

::

   ros2 run dvrk_console stereo_display_calibration

For display layout details, see :ref:`dvrk-stereo-display-configuration`.


.. _app-gscam-socket:

``gscam_socket``
================

* Python bridge script from ``dvrk_data``
* Bridges the high-speed local ``unixfd`` socket stream to standard ROS 2 ``sensor_msgs/msg/Image`` topics via gscam
* https://github.com/jhu-dvrk/dvrk_data

::

   ros2 run dvrk_data gscam_socket

For bridging details, see :ref:`dvrk-video-data`.
