.. _dvrk-data:

``dvrk_data``
#############

``dvrk_data`` provides the video and experiment-data foundation for the dVRK.
It deliberately separates the high-bandwidth local video path from ROS 2
robot messages.

Video data
**********

The video applications form a modular GStreamer pipeline.  Camera acquisition,
stereo alignment, display, recording, and ROS bridging can run in separate
processes while sharing local buffers through ``unixfd`` sockets.  Frame
metadata carries timing observations through these stages.

Start with :doc:`video-data`, then use :doc:`video-configuration` and
:doc:`timestamps` as references.

Data collection tools
*********************

The collection workflow combines MP4 video and per-frame timestamp sidecars
with optional audio and ROS 2 bags.  The principal tools are :doc:`record`,
:doc:`extract`, and :doc:`latency-calibration`.  :doc:`tagging` describes the
optional curation tools.

.. toctree::
   :maxdepth: 1

   video-data
   video-configuration
   timestamps
   data-collection
   record
   extract
   latency-calibration
   tagging
