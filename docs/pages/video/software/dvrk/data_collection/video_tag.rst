Video Tag
=========

The ``video_tag`` C++ application is designed for post-recording data curation. It allows users to review recorded videos produced by :doc:`record` with frame-accurate precision and assign temporal labels (stages) or discrete frame tags.

.. note::
    When tagging a session with multiple video streams, users should select and use a **primary video** to perform the tagging. This primary video acts as the reference for temporal alignment across all other streams.

Usage
-----

The tool requires the session's configuration file and a specific video file from the recording session.

.. code-block:: bash

    ros2 run data_collection video_tag -v 2026/02/18_025457/camera_1_220117_153206.mp4 -c config.json

The application will strictly check for the existence of all provided parameters and abort with a CRITICAL error if the video or configuration file is missing.

The application will automatically look for and load any existing ``tags.json`` or metadata from the session directory.

Features
--------

- **Frame-Accurate Navigation:** Navigation is based strictly on recorded frame indices from the sidecar JSON file.
- **Virtual Time Display:** Time is displayed as "recorded time" (min:sec.ms), skipping any unrecorded gaps.
- **Lead-in Skipping:** Automatically skips unrecorded lead-in video to start on the first recorded frame.
- **Stage Support:** Labels defined in the config are automatically shown as toggleable ranges.
- **Tag Search:** Jump quickly between frame tags using dropdown menus.
- **Missing Tag Validation:** Automatically detects tags in the JSON file that are missing from the current configuration and asks to "Accept" them.
- **Speed Control:** Variable playback speeds (0.1x to 2.0x).
- **Session Tags:** Can automatically load tags from the session's ``tags.json`` to overlay existing annotations.
