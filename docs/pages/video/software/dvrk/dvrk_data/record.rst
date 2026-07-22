Record
######

``record`` is the collection entry point.  It previews multiple GStreamer
streams, records selected streams to H.264 MP4, optionally records audio, and
writes selected ROS 2 topics to a bag in the same session directory.

Run it with one or more versioned record configurations:

.. code-block:: bash

   ros2 run dvrk_data record -c record.json

Useful options are:

.. code-block:: text

   -c <file>           add a configuration file
   -j <threads>        set the maximum encoding worker count
   -p <topic>          change the start/stop trigger topic
   -g <0|1|2|3>       write GStreamer pipeline graphs at the selected detail

Configuration
*************

This minimal configuration records the aligned stereo socket and two ROS
topics:

.. code-block:: json

   {
     "type": "dvrk_data:record@1.0.0",
     "data_directory": "data",
     "videos": [
       {
         "name": "stereo",
         "socket": "stereo_alignment:stereo",
         "record": true,
         "side_by_side": "LR",
         "encoding": {
           "bitrate_kbps": 20000,
           "frame_rate": 30
         }
       }
     ],
     "ros_topics": [
       {"name": "/PSM1/measured_cp", "continuous": true},
       {"name": "/console/clutch", "continuous": false}
     ]
   }

For a video entry, ``socket`` takes precedence over ``stream``.  A direct
``stream`` is a GStreamer source fragment.  ``side_by_side`` records whether
the first half is the left eye (``LR``) or right eye (``RL``), allowing
:doc:`extract` to split it later.

The recorder probes NVENC, NVIDIA V4L2, and VAAPI encoders before falling back
to ``x264enc``.  Availability of a plugin alone does not guarantee that the
installed driver can initialize it, so hardware candidates are tested at
runtime.

Configuration composition
=========================

``configuration_files`` recursively includes other
``dvrk_data:record@1.0.0`` documents.  Relative paths are searched from the
including file, the installed ``dvrk_data`` share directory, and the top-level
configuration directory.  Circular or duplicate references are loaded once.

Video entries and stages are appended.  ROS topics are deduplicated by name.
This makes it practical to maintain reusable arm, console, camera, and
experiment-stage files.

ROS topic timing
================

Each ``ros_topics`` item has a ``name`` and optional ``continuous`` flag.
Non-continuous topics are written only while recording is active.  Continuous
topics begin when the first acquisition starts and continue for the remainder
of the session, including gaps between video recordings.

Operation and stages
********************

The GTK interface controls all configured streams and shows live recording
statistics.  Optional ``stages`` provide an ordered experiment sequence; the
selected stage is included in output names and advances after a recording.
Optional event ``tags`` can be applied during acquisition.

The default ROS controls are:

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
     - Purpose
   * - ``record/record``
     - ``std_msgs/msg/Bool``
     - Start with ``true`` and stop with ``false``.
   * - ``record/recording``
     - ``std_msgs/msg/Bool``
     - Reports the actual recording state.

For example:

.. code-block:: bash

   ros2 topic pub /record/record std_msgs/msg/Bool "{data: true}" --once
   ros2 topic pub /record/record std_msgs/msg/Bool "{data: false}" --once

Session output
**************

A session contains an ``index.json``, MP4 files and matching versioned sidecar
JSON files, an optional ROS bag, and optional audio and tag files.  Each video
sidecar records:

* all observed timestamps and their clock domains;
* the preferred capture time chosen for correlation;
* timestamp provenance statistics;
* frames recorded and queue overruns;
* stereo eye order and calibrated latency;
* the configuration files used for the session.

See :doc:`timestamps` for the meaning of individual fields.
