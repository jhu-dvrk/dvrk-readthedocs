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

Use :ref:`dvrk-latency-calibration` before the final experiment when an empirical
video latency estimate should be recorded with the session.  The measured
value is stored in the same video entry consumed by :ref:`dvrk-record`.

The applications do not force every source onto a shared clock or trigger.
They preserve source-specific observations and use explicit frame timestamps
for extraction.  Hardware synchronization, PTP clock alignment, empirical
latency compensation, or experiment-specific correlation signals remain
separate concerns.

The principal tools are:

* :ref:`dvrk-record` for acquisition and session metadata.
* :ref:`dvrk-extract` for images, video ranges, and ROS-topic CSV files.
* :ref:`dvrk-latency-calibration` for empirical video latency.
* :ref:`dvrk-tagging` for optional post-recording curation.
