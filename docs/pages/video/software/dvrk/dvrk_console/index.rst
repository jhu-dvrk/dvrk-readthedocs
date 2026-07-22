.. _dvrk-console:
.. _stereo-viewer:

``dvrk_console``
################

``dvrk_console`` contains the applications intended for routine use at the
surgeon console:

* :doc:`stereo-display` renders the aligned endoscope stream, dVRK status HUD,
  optional picture-in-picture and AR content, and includes the display
  calibration workflow.
* :doc:`control-panel` provides the operator's routine robot and teleoperation
  controls, status, and an optional embedded video preview.

These applications consume lightweight ROS 2 robot state and commands, while
video remains in local GStreamer pipelines.  The robot control process may run
on another computer, but ``unixfd`` video producers and consumers must share
the display computer.

.. toctree::
   :maxdepth: 1

   stereo-display
   stereo-display-configuration
   control-panel
