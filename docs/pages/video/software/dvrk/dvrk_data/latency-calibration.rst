Video latency calibration
#########################

``video_latency`` estimates capture-to-observation latency for one direct
GStreamer stream in a record configuration.  It alternates black and white
display transitions, detects the corresponding change in the camera image,
and measures elapsed time with a monotonic clock.

Select the video by its configured name:

.. code-block:: bash

   ros2 run dvrk_data video_latency -c record.json -s endoscope

Running without ``-s`` lists available names and exits.  The selected entry
must currently provide ``stream``; the latency tool does not construct its
input from the recorder's ``socket`` field.

Procedure
*********

#. Frame a surface on which the calibration window is visible to the camera.
#. Wait for the displayed level to settle away from fully black or white.
#. Select **Estimate Latency**.
#. The tool alternates black and white, collecting ten successful transitions.
#. Review the mean and standard deviation.
#. Choose whether to write the mean to the video's ``estimated_latency`` field.

The configuration stores seconds.  :doc:`record` copies the value to the video
sidecar as ``estimated_latency_ms`` and :doc:`extract` applies it when
correlating acquisition ranges.

This is an empirical end-to-end estimate for the tested arrangement.  Display
refresh, exposure, capture buffering, and pipeline processing all contribute,
so recalibrate after changing relevant hardware or pipeline elements.
