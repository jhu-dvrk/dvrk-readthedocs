Data collection workflow
########################

The collection tools are designed as a sequence:

.. code-block:: text

   record configuration
           |
           v
        record ------> MP4 + sidecar JSON + ROS bag + session index
           |                              |
           +----------> optional tags ----+
                                          |
                                          v
                                       extract
                                          |
                             timestamped media and CSV

Use :doc:`latency-calibration` before the final experiment when video must be
correlated closely with robot telemetry.  The measured value is stored in the
same video entry consumed by :doc:`record`.

The applications do not force every source onto a shared clock or trigger.
They preserve source-specific observations and apply declared latency
corrections during extraction.  Hardware synchronization, PTP clock alignment,
or experiment-specific correlation signals remain separate concerns.

The principal tools are:

* :doc:`record` for acquisition and session metadata.
* :doc:`extract` for images, video ranges, and ROS-topic CSV files.
* :doc:`latency-calibration` for empirical video latency.
* :doc:`tagging` for optional post-recording curation.
