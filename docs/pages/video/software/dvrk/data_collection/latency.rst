Tools for Latency
=================

This section covers tools used to measure and verify timing alignment and latency.

Video Latency
-------------

The ``video_latency`` tool (see :doc:`record`'s configuration for defining streams) provides a simple automated way to estimate the end-to-end latency of a video stream (from capture to display/processing).

It works by:
1. Displaying a live preview of the video stream.
2. Flashing the screen white at randomized intervals.
3. Measuring the time delta between the software-triggered flash and its appearance in the video stream.

To run the latency estimation:

.. code-block:: bash

    ros2 run data_collection video_latency -c config.json

Once the estimated latency is identified and saved in the configuration file, it can be used during extraction to adjust timestamps and improve alignment with ROS topics and other data sources.

Checking Embedded Timestamps
----------------------------

.. note::
   The embedded timestamp verification is currently **experimental** and operates on frames extracted by :doc:`extract`.

The ``check_timestamps.py`` script (located in the ``tests/`` directory) verifies timestamp consistency between recording filenames (based on system time) and burned-in GStreamer timestamps. It uses Tesseract OCR to read the "Timestamp overlay" strip.

To verify timestamps in extracted frames:

.. code-block:: bash

    python3 tests/check_timestamps.py -d 20260117_153206/extracted

**Key Features:**

- **Automatic ROI:** Specifically targets the bottom 30px black timestamp overlay strip for speed and accuracy.
- **Latency Analysis:** Calculates the average difference between system capture time and the video's internal clock.
- **Jitter Measurement:** Calculates the standard deviation of latency across all frames.
- **Transition Detection:** Identifies frame boundaries where the integer second changes to estimate sub-second precision.
- **Validation:** Filters out OCR misreads and handles logical day-wrapping.
