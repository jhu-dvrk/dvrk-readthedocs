.. _dvrk-data:

dVRK Data
=========

.. warning::
   The dVRK data collection suite is exclusively supported on ROS 2. ROS 1 is **not** supported.

The ``dvrk_data`` package has two related roles:

* It provides GStreamer video utilities for acquisition, alignment, transport,
  and inspection.  These utilities are designed around multiple cooperating
  processes that exchange frames through ``unixfd`` sockets instead of routing
  high-bandwidth video through ROS topics.
* It provides data collection tools for synchronized experiments: video
  recording, ROS 2 bag recording, session metadata, video tagging, timestamp
  inspection, and extraction into files suitable for analysis or training.

The video path preserves as much timing information as possible.  Hardware
streams are captured directly with GStreamer, frames carry source timestamps,
and recorded videos are paired with sidecar timestamp files.  ROS 2 messages can
be recorded alongside video, but the package does not force all data streams
through a single synchronized transport while recording.

It is important to note that the data collection tools explicitly **do not**
synchronize the disparate video and topic streams together natively. Instead,
during extraction, individual per-source ``estimated_latency`` values are used
alongside accurate timestamps to align the sampled data together as closely as
chronologically possible.

Package Layout
--------------

Video transport and utility executables include ``stereo_source``,
``stereo_alignment``, ``stereo_alignment_calibration``,
``video_configurator``, ``gscam_socket``, and ``gscam.launch.py``.  They can be
used to build local GStreamer pipelines, publish selected streams through
``@dvrk_gst`` abstract Unix sockets, calibrate stereo alignment, or bridge a
stream back into ROS when needed.  Use ``gscam_socket`` to list active sockets
and launch a ``gscam_node`` with a single command.

Data collection executables include ``record``, ``video_tag``,
``video_latency``, ``extract``, and ``encord_to_tags``.  These tools share JSON
configuration files and session metadata so a recording can be reviewed,
annotated, and extracted without losing the original video timestamps.

Data Collection Workflow
------------------------

The suite of tools provided by the ``dvrk_data`` package is designed to work together in a sequence. This section describes the data flow and how shared files connect the different programs.

Data Flow Overview
~~~~~~~~~~~~~~~~~~

1. **Configuration**: A master JSON configuration file defines video sources, ROS 2 topics, optional stages, and recording parameters.
2. **Recording**: The :doc:`record` application captures video, audio, and ROS 2 topics while preserving per-source timestamps.
3. **Curation**: The :doc:`video_tag` tool loads the session to add temporal labels and frame-accurate tags.
4. **Extraction**: The :doc:`extract` tool processes the recorded videos and ROS 2 bags to generate analysis-ready data such as images, videos, and CSVs.
5. **Validation**: The :doc:`latency` tools verify timing consistency and measure system performance.

Shared Configuration Files
~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``config.json``: The primary configuration file used by :doc:`record <record>`, :doc:`video_tag <video_tag>`, and ``video_latency`` (in :doc:`latency`). It ensures all tools use the same video stream definitions, labels, latency estimates, and stage names.
- ``JSON Schema``: Defines the structure of the configuration files, facilitating reuse and composition.

Session Directory Output
~~~~~~~~~~~~~~~~~~~~~~~~

When :doc:`record <record>` completes a session, it creates a directory (named by its timestamp, e.g., ``2026/02/18_025457/``) containing:

- **Video Files**: Recorded ``.mp4`` files for each enabled camera.
- **Sidecar Timestamps**: JSON files named ``camera_name_YYMMDD_HHMMSS_timestamps.json`` containing nanosecond-accurate epoch timestamps for every frame.
- **ROS bags**: Recorded data topics stored in ROS 2 bag format.
- **index.json**: A summary of durations, metadata, and data paths.

Downstream Consumption
~~~~~~~~~~~~~~~~~~~~~~

The session directory and its contents are consumed by:

- :doc:`video_tag <video_tag>`: Loads the video files and uses ``index.json`` to identify available streams. It generates or updates a ``tags.json`` file in the session directory.
- :doc:`extract <extract>`: Uses the sidecar timestamps to extract frames from ``.mp4`` at the exact original nanoseconds and converts ROS bags into easy-to-use CSVs.
- :doc:`check_timestamps <latency>`: Uses extracted frames to verify timestamp consistency and quantify timing offsets against burned-in overlays.

.. toctree::

   installation
   record
   video_tag
   extract
   latency

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
