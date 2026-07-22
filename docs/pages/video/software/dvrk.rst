.. _dvrk-video-software:

dVRK video software
###################

The dVRK video stack is divided into two packages.  ``dvrk_data`` owns
video acquisition, local transport, stereo alignment, timing, recording, and
extraction.  ``dvrk_console`` consumes those streams for the surgeon display
and provides a simplified operator control panel.

The normal video path keeps large image buffers in GStreamer and uses Linux
``unixfd`` sockets between processes.  ROS 2 carries robot state, commands,
and recorded telemetry; image topics are added only when a ROS image consumer
requires them.

.. code-block:: text

   cameras
      |
      v
   stereo_source -- left/right unixfd --> stereo_alignment
                                               |
                                      aligned stereo unixfd
                                               |
                         +---------------------+------------------+
                         |                     |                  |
                       record           stereo_display     other consumers
                                               |
                                         control_panel
                                         optional preview

.. warning::

   These packages support ROS 2 only.  ROS 1 is not supported.

.. toctree::
   :maxdepth: 2

   dvrk/dvrk_data/index
   dvrk/dvrk_console/index
