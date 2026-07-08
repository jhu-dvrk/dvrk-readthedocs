Record
======

The ``record`` application is the data collection entry point in
``dvrk_data``.  It captures multi-stream video and audio through GStreamer,
preserves each source's native timestamps, and can record ROS 2 topics into
bags during the same session.

Video acquisition is intentionally independent of ROS image topics.  Video
sources are described as GStreamer snippets, and streams can come directly from
hardware or from another local process through ``unixfdsrc``.  This keeps the
high-bandwidth image path local to GStreamer while ROS 2 is used for command,
status, and lower-bandwidth robot messages.

Examples and configuration files
--------------------------------

Define your video sources, ROS 2 topics, and optional stages in a JSON file.
The configuration format is defined in the JSON schema.

**Example config.json:**

.. code-block:: json

    {
      "data_directory": "data",
      "record_audio": true,
      "ros_topics": [
        "/PSM1/measured_cp",
        "/PSM1/measured_cv",
        "/PSM1/jaw/measured_js"
      ],
      "stages": [
        "calibration",
        "exercise_1",
        "exercise_2"
      ],
      "videos": [
        {
          "name": "camera_1",
          "stream": "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1",
          "record": true,
          "timestamp_overlay": true,
          "encoding": {
            "width": 320,
            "height": 240,
            "bitrate": 5000
          }
        },
        {
          "name": "test_pattern",
          "stream": "videotestsrc pattern=smpte75",
          "record": false
        }
      ]
    }

Stereo Video Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

For side-by-side stereo video sources, including streams created by another
GStreamer process and delivered through ``unixfd``, use the ``side_by_side``
field to indicate the layout:

.. code-block:: json

    {
      "videos": [
        {
          "name": "stereo_camera",
          "stream": "... compositor pipeline combining left and right sources ...",
          "side_by_side": "LR",
          "record": true
        }
      ]
    }

- ``"LR"``: Left eye on the left half, right eye on the right half.
- ``"RL"``: Right eye on the left half, left eye on the right half.

This metadata is stored in the sidecar JSON and enables :doc:`extract` to split the video into separate left/right channels using the ``-S`` option.

Stereo Alignment Utility
~~~~~~~~~~~~~~~~~~~~~~~~

Use the ``stereo_alignment_calibration`` utility to interactively tune a stereo
stream and save alignment values in a JSON config.  This utility belongs to the
``dvrk_data`` transport layer; the surgeon-console display calibration is
documented separately with ``dvrk_console``.

.. code-block:: bash

    ros2 run dvrk_data stereo_alignment_calibration -c stereo_alignment.json \
      -l "v4l2src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=30/1" \
      -r "v4l2src device=/dev/video1 ! video/x-raw,width=1280,height=720,framerate=30/1"

The tool provides:

- Keyboard controls only (OpenCV window), no Tk/GUI dependency.
- Inputs must be GStreamer source snippets for both ``--left`` and ``--right``.
- **+/-**: resize the crop rectangle (aspect ratio preserved, no scaling stage in pipeline).
- **Left/Right arrows**: change horizontal baseline in pixels.
- **Up/Down arrows**: change vertical offset between the left and right images.
- **f**: toggle fullscreen preview and stretch to fill the screen (preview only).

The saved alignment values can be reused with the same left and right stream definitions, for example:

.. code-block:: json

    {
      "type": "dvrk_data:stereo_alignment@1.0.0",
      "name": "stereo_alignment",
      "camera": {
        "size": { "width": 1280, "height": 720 },
        "left": { "stream": "v4l2src device=/dev/video0" },
        "right": { "stream": "v4l2src device=/dev/video1" },
        "crop": { "width": 1200, "height": 675 },
        "alignment": { "horizontal_shift_px": 0, "vertical_shift_px": 0 }
      }
    }

The initial resize is 100% (no zoom).

Configuration File Composition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Configuration files can reference other configuration files using the ``configuration_files`` field. This allows you to organize and reuse configurations across different setups.

**Example with configuration_files:**

.. code-block:: json

    {
      "data_directory": "data",
      "configuration_files": [
        "devices/PSM1.json",
        "devices/PSM2.json",
        "cameras/stereo.json"
      ]
    }

**How it works:**

- Referenced configuration files are loaded and merged recursively.
- Paths in ``configuration_files`` are resolved relative to the current config file's directory.
- If a file is not found relative to the current config, it falls back to searching relative to the master config file's directory.
- All ``videos``, ``ros_topics`` and ``stages`` from referenced files are combined (deduplicated).
- The ``data_directory`` from the last processed file is used.
- Circular dependencies are automatically detected and prevented.

Running the Record
~~~~~~~~~~~~~~~~~~

After building your workspace, run the record using ``ros2 run``:

.. code-block:: bash

    ros2 run dvrk_data record -c config.json

**Note:** Configuration file paths can be relative to your current working directory or absolute paths.
The application will strictly check for the existence of all provided configuration files and abort with a CRITICAL error if any are missing.

Multiple configuration files can be loaded and merged. You can collect multiple video streams and multiple ROS topics defined in existing files (e.g. ``PSM1.json``). This allows users to re-use configuration files for each component used for a given experimental setup.

Stages Feature
~~~~~~~~~~~~~~

If the ``stages`` field is provided in the configuration, a "Stages" list will appear on the right side of the GUI.

- **File Naming:** When a stage is selected, its name is appended to the session directory and all recorded files. The naming convention for video files is ``camera_name_YYMMDD_HHMMSS_stage.mp4``.
- **Auto-Advancement:** After stopping a recording, the application automatically selects the next stage in the list.
- **Looping:** When the last stage is completed, it wraps back to the first stage.
- **Manual Override:** Users can click any stage in the list to select it for the next recording (selection is disabled while recording is in progress).
- **Hardware-Accelerated Encoding:** Automatically detects and uses available hardware encoders (NVENC, VAAPI) to minimize CPU usage.
- **Nanosecond Precision:** All video frames are timestamped in nanoseconds since epoch. These timestamps are stored in sidecar JSON files and used by :doc:`extract` together with per-source ``estimated_latency`` to align data as closely as possible.
- **Session Metadata:** An ``index.json`` file is created in each session directory, storing video/bag durations and metadata. This metadata is also consumed by :doc:`video_tag`.

ROS integration
---------------

The record functions as a ROS 2 node named ``record``.

Control Topics
~~~~~~~~~~~~~~

====================== ====================== ============ =====================================================
Topic                  Type                   Direction    Description
====================== ====================== ============ =====================================================
``record/record``      ``std_msgs/msg/Bool``  Subscriber   Send ``true`` to start recording, ``false`` to stop.
``record/recording``   ``std_msgs/msg/Bool``  Publisher    Publishes actual recording state (``true`` if recording).
====================== ====================== ============ =====================================================

Command Line Examples
~~~~~~~~~~~~~~~~~~~~~

Ensure you have sourced your ROS 2 environment (e.g., ``source /opt/ros/humble/setup.bash``).

**Start Recording:**

.. code-block:: bash

    ros2 topic pub /record/record std_msgs/msg/Bool "{data: true}" --once

**Stop Recording:**

.. code-block:: bash

    ros2 topic pub /record/record std_msgs/msg/Bool "{data: false}" --once

**Monitor Status:**

.. code-block:: bash

    ros2 topic echo /record/recording
