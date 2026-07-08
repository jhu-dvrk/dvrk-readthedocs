dVRK software
=============

The ROS 2 dVRK video tools are split into two packages with distinct roles:

* ``dvrk_data`` provides the video transport and data collection tools.  It
  focuses on GStreamer-based acquisition, multi-process video routing with
  ``unixfd`` sockets, timestamp preservation, session recording, video tagging,
  extraction, and ROS 2 bag capture.
* ``dvrk_console`` provides applications intended for the surgeon's console:
  ``stereo_display`` for the HRSV/stereo display pipeline and
  ``control_panel`` for a simplified operator-facing UI.  This is separate from
  the main ``dvrk_system`` UI, which remains the debug/engineering interface.

.. toctree::

   dvrk/viewer/index
   dvrk/dvrk_data/index
