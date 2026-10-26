Data collection workflow
########################

The collection tools follow this workflow:

.. figure:: /images/software/data-collection.svg
   :width: 100%
   :align: center
   :alt: Video streams, ROS 2 topics, and recording settings feed dvrk_data record. The recorded session can be extracted in full, or video_tag can create frame ranges for tagged extraction. Extraction produces timestamped media and ROS-topic CSV files.

   ``record`` writes a session that ``extract`` can process in full. Optionally,
   ``video_tag`` marks ranges in a recorded video for selected extraction. The
   :download:`Graphviz source </images/software/data-collection.dot>` is
   available with the diagram.

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
