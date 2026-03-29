.. _data-collection:

Data Collection
===============

.. warning::
   The dVRK data collection suite is exclusively supported on ROS 2. ROS 1 is **not** supported.

A multi-stream video record application using C++, GStreamer, and GTKmm. It allows creating flexible video streams via JSON configuration, provides live previews, and preserves per-source timestamps during recording. ROS topics can also be recorded alongside videos. 

It is important to note that the data collection nodes explicitly **do not** synchronize the disparate video and topic streams together natively. Instead, during extraction, individual per-source ``estimated_latency`` values are used alongside accurate timestamps to aggressively **align** the sampled data together as closely as chronologically possible. Furthermore, all videos are cleanly recorded directly from the hardware source using GStreamer completely bypassing ROS topics for latency integrity.


The application also integrates with ROS 2 for remote control and status monitoring.

Workflow and Shared Files
-------------------------

The suite of tools provided in this repository is designed to work together in a sequence. This section describes the data flow and how shared files connect the different programs.

Data Flow Overview
~~~~~~~~~~~~~~~~~~

1. **Configuration**: A master JSON configuration file defines the hardware setup and recording parameters.
2. **Recording**: The :doc:`record` application uses the configuration to capture video, audio, and ROS topics.
3. **Curation**: The :doc:`video_tag` tool loads the session to add temporal labels and frame-accurate tags.
4. **Extraction**: The :doc:`extract` tool processes the recorded videos and ROS bags to generate training-ready data (images and CSVs).
5. **Validation**: The :doc:`latency` tools verify timing consistency and measure system performance.

Shared Configuration Files
~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``config.json``: The primary configuration file used by :doc:`record <record>`, :doc:`video_tag <video_tag>`, and ``video_latency`` (in :doc:`latency`). It ensures all tools use the same video stream definitions and labels.
- ``JSON Schema``: Defines the structure of the configuration files, facilitating reuse and composition.

Session Directory Output
~~~~~~~~~~~~~~~~~~~~~~~~

When :doc:`record <record>` completes a session, it creates a directory (named by its timestamp, e.g., ``2026/02/18_025457/``) containing:

- **Video Files**: Recorded ``.mp4`` files for each enabled camera.
- **Sidecar Timestamps**: JSON files named ``camera_name_YYMMDD_HHMMSS_timestamps.json`` containing nanosecond-accurate epoch timestamps for every frame.
- **ROS bags**: Recorded data topics stored in ROS2 bag format.
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
