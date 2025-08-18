Introduction
############

If you need to process the video inputs (e.g. camera calibration,
filtering, 3D reconstruction...) or modify the videos being displayed
(camera distortion compensation, overlays, messages...) you will need
to put a PC in the middle of the video pipeline.  In this case, the
main 3 "components" on the PC are:

* Frame grabbers
* Software
* Video output (most likely a graphic card with 2 extra outputs for HRSV)

.. figure:: /images/video/video-pipeline-software.*
   :width: 600
   :align: center

   Software based video pipeline
