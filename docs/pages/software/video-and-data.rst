.. _video_and_data:
.. _video-and-data:
.. _dvrk-video-software:

##############
Video and data
##############

The dVRK video stack provides low-latency stereo video
acquisition, surgeon console displays, operator controls, and multi-modal data recording.

The stack is organized into two ROS 2 packages:

``dvrk_data``
  Owns camera acquisition, local video transport, real-time stereo alignment,
  per-frame hardware/software timestamping, data recording, and dataset
  extraction.

``dvrk_console``
  Owns the surgeon console display engine (supporting Classic HRSV, 3D
  monitors, and headsets), vector head-up display (HUD) overlays,
  picture-in-picture (PiP) and augmented reality (AR) mixing, and the operator
  touchscreen/desktop control panel.

.. warning::

   These packages support **ROS 2 only**.  ROS 1 is not supported.


Architecture and design choices
*******************************

The Teleoperation Latency Challenge
===================================

Stereo High-Definition video (e.g. 1080p60 per eye) generates uncompressed raw
pixel throughput exceeding **3 Gbps** (over 370 MB/s). Passing this volume of
imagery through a general-purpose robotic middleware introduces significant
bottlenecks:

* **Serialization and Copies**: Standard ROS 2 ``sensor_msgs/msg/Image`` topics
  typically require memory allocations, deep copies, and middleware serialization
  at every publisher and subscriber.
* **Buffer Pressure**: High-rate image topics saturate DDS ring buffers and CPU
  caches, causing variable scheduling jitter and frame drops.
* **Queue Lag**: When downstream consumers (such as video recorders or UI widgets)
  experience momentary slowdowns, middleware queues can buffer frames, causing
  the live display to fall progressively behind real time.

Separation of video data and control data
===========================================

To keep video latency low alongside robot integration, the dVRK stack separates
two data paths:

1. **The video data (GStreamer + Linux Abstract Sockets)**:
   All high-bandwidth video frame movement remains strictly within local
   GStreamer pipelines. Between separate processes on the same machine, frames
   are passed through Linux domain sockets using ``unixfdsink`` and
   ``unixfdsrc``. This transfers buffer file descriptors (such as shared
   memory or DMA buffers) through the kernel. Sharing these buffers avoids ROS
   image serialization; format conversion, scaling, compositing, or encoding
   can still copy or transform pixels.
2. **The control data (ROS 2 & CRTK)**:
   ROS 2 carries lightweight, low-bandwidth control messages: robot joint
   kinematics, Cartesian tool poses, foot pedal events (clutch, camera control),
   teleoperation states, and system warnings.

The live source, alignment, and display paths use bounded **leaky queues**
(``leaky=downstream``) and one cached image per mixer input. An update from either
eye uses the newest available images, without waiting for a matching timestamp
or the other eye. This behavior is unconditional, including display shifts,
picture-in-picture, and AR layouts. See :ref:`dvrk-latest-frame-policy`.

The display's :ref:`dvrk-frame-timing` panel reports per-eye age, source update
rate, and channel time difference so repeated or stalled video remains visible.

When standard ROS image consumers (such as computer vision nodes or remote
viewers) are needed, an on-demand bridge (``gscam_socket``) can expose selected
streams as standard ROS 2 topics without affecting the critical live path.


System topology and data flow
*****************************

The complete video and data pipeline is decoupled into specialized processes
communicating over local abstract sockets:

.. figure:: /images/software/video-and-data.svg
   :width: 100%
   :align: center
   :alt: Left and right cameras feed stereo_source, then stereo_alignment. Aligned video branches to stereo_display and record. The display drives a physical surgeon display and publishes an optional overlay preview. ROS 2 carries console status and operator commands separately from the video sockets.

   Live stereo video follows GStreamer sockets; ROS 2 carries console state and
   operator commands. The control panel can preview the source, aligned, or
   overlaid video. The :download:`Graphviz source
   </images/software/video-and-data.dot>` is available with the diagram.


The ``@dvrk`` socket convention
*******************************

Inter-process video sockets use the **Linux abstract namespace** with the
canonical format:

.. code-block:: text

   @dvrk:<role>:<name>

Abstract sockets have several advantages over filesystem paths (such as ``/tmp/*.sock``):

* **No Orphaned Files**: Abstract sockets are managed purely in kernel memory.
  If an application crashes or is terminated unexpectedly, the operating system
  automatically cleans up the socket immediately.
* **Deterministic Addressing**: Standardized roles ensure predictable endpoints:

  * ``@dvrk:stereo_source:stereo`` (default combined LR source)
  * ``@dvrk:stereo_source:left`` and ``@dvrk:stereo_source:right`` (when configured)
  * ``@dvrk:stereo_alignment:stereo`` (default aligned output)
  * ``@dvrk:stereo_display:overlay`` (default display socket output; configurable)
* **Discoverability**: The active dVRK socket registry can be queried in real
  time using ``ros2 run dvrk_data gscam_socket``.


Timing observations and frame selection
***************************************

The live pipeline selects images by arrival, independently for each eye. It
preserves timestamps for diagnostics and later correlation, without using them
to pair or delay left and right frames.

#. ``stereo_source`` records CPU ``CLOCK_REALTIME`` observations as frames pass
   its source probes. These are Unix-epoch wall-clock values, not monotonic or
   hardware exposure times.
#. Per-eye source observations and available hardware/reference-clock metadata
   follow the selected pixels through alignment and display. Output-stage times
   describe publication and processing, not a replacement camera observation.
#. ``record`` writes frame indices and explicit timing observations to
   ``dvrk_data:video_sidecar@1.0.0`` JSON sidecars beside the MP4 files.
#. These observations support video/robot-data correlation. Hardware triggering,
   clock alignment across devices, and empirical camera latency remain separate
   concerns; recording timestamps does not itself synchronize sensors.

See :ref:`dvrk-timestamps` for clock domains and missing stage times, and
:ref:`dvrk-frame-timing` for the live display statistics.


Documentation structure
***********************

.. toctree::
   :maxdepth: 2

   video-and-data/dvrk_data/index
   video-and-data/dvrk_console/index

