.. _video_and_data:
.. _video-and-data:
.. _dvrk-video-software:

##############
Video and data
##############

The dVRK video stack provides low-latency stereo video
acquisition, surgeon console displays, operator controls, and multi-modal data recording.

The stack is organized into two ROS 2 packages:

* **``dvrk_data``**: Owns camera acquisition, local video transport, real-time
  stereo alignment, per-frame hardware/software timestamping, data recording,
  and dataset extraction.
* **``dvrk_console``**: Owns the surgeon console display engine (supporting
  Classic HRSV, 3D monitors, and headsets), vector head-up display (HUD)
  overlays, picture-in-picture (PiP) and augmented reality (AR) mixing, and the
  operator touchscreen/desktop control panel.

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

To achieve guaranteed sub-frame display latency alongside rich robot integration,
the dVRK stack cleanly separates two data paths:

1. **The video data (GStreamer + Linux Abstract Sockets)**:
   All high-bandwidth video frame movement remains strictly within local
   GStreamer pipelines. Between separate processes on the same machine, frames
   are passed through Linux domain sockets using **``unixfdsink``** and
   **``unixfdsrc``**. This transfers buffer file descriptors (such as shared
   memory or DMA buffers) directly through the Linux kernel without copying image
   payloads.
2. **The control data (ROS 2 & CRTK)**:
   ROS 2 carries lightweight, low-bandwidth control messages: robot joint
   kinematics, Cartesian tool poses, foot pedal events (clutch, camera control),
   teleoperation states, and system warnings.

Small, bounded **leaky queues** (``leaky=downstream``) are placed at each socket
boundary. If a recording sink or monitor lags momentarily, old frames are
immediately dropped so that the live surgeon viewer always displays the freshest
possible image.

When standard ROS image consumers (such as computer vision nodes or remote
viewers) are needed, an on-demand bridge (``gscam_socket``) can expose selected
streams as standard ROS 2 topics without affecting the critical live path.


System topology and data flow
*****************************

The complete video and data pipeline is decoupled into specialized processes
communicating over local abstract sockets:

.. code-block:: text

   +-------------------------------------------------------------+
   |                       Camera Hardware                       |
   |              (DeckLink SDI, V4L2 HDMI/USB, etc.)            |
   +-------------------------------------------------------------+
                                  |
                                  v
   +-------------------------------------------------------------+
   |                     stereo_source (C++)                     |
   |  - Left and right hardware frame capture                    |
   |  - Real-time timestamp probe (CLOCK_REALTIME)               |
   |  - Converts to I420 and exposes bounded output sockets       |
   +-------------------------------------------------------------+
             |                                        |
             | @dvrk:stereo_source:left               | @dvrk:stereo_source:right
             +--------------------+-------------------+
                                  |
                                  v
   +-------------------------------------------------------------+
   |                    stereo_alignment (C++)                   |
   |  - Per-eye color correction, crop, and geometric shifts     |
   |  - Composes aligned side-by-side stereo stream              |
   |  - Adds stereo_output timestamp to frame metadata           |
   +-------------------------------------------------------------+
                                  |
                                  | @dvrk:stereo_alignment:stereo
            +---------------------+---------------------+
            |                                           |
            v                                           v
   +-----------------------------+       +-----------------------------+
   |     stereo_display (C++)    |       |         record (C++)        |
   |  - Low-latency OpenGL sinks |       |  - Multi-stream MP4 video   |
   |  - Cairo vector HUD overlays|       |  - JSON timestamp sidecars  |
   |  - AR & PiP stream blending |       |  - Synchronized MCAP bag    |
   |  - HRSV / 3D monitor / HMD  |       +-----------------------------+
   +-----------------------------+
            |
            | (Subscribes to ROS 2 CRTK topics:
            |  arm states, clutch, camera pedals)
            v
   +-----------------------------+
   |      control_panel (C++)    |
   |  - Touchscreen operator GUI |
   |  - Robot power, homing, teleop
   |  - Optional embedded preview|
   +-----------------------------+


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
  - ``@dvrk:stereo_source:left`` and ``@dvrk:stereo_source:right``
  - ``@dvrk:stereo_alignment:stereo``
  - ``@dvrk:stereo_display:stereo`` (optional composed display loopback)
* **Discoverability**: The active dVRK socket registry can be queried in real
  time using ``ros2 run dvrk_data gscam_socket``.


Timing and synchronization
**************************

Good surgical data science and robot learning require precise
time stamping across heterogeneous sensors:

1. **Hardware/Source Timestamping**: As each frame arrives at ``stereo_source``,
   a GStreamer buffer probe captures a monotonic ``CLOCK_REALTIME`` timestamp.
2. **Metadata Propagation**: This timestamp travels alongside the buffer through
   alignment and display stages via custom GStreamer metadata structures.
3. **Per-Frame Sidecars**: During recording, ``dvrk_data record`` writes a
   compact JSON sidecar (``video_sidecar@1.0.0``) storing frame index,
   pipeline duration, and precise epoch timestamps alongside the compressed MP4
   file.
4. **Multi-Modal Correlation**: Video frame timestamps correlate directly with
   robot kinematic and wrench samples recorded in parallel ROS 2 MCAP rosbags.


Documentation structure
***********************

.. toctree::
   :maxdepth: 2

   video-and-data/dvrk_data/index
   video-and-data/dvrk_console/index

