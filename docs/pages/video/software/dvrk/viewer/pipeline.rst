Integration & Pipeline
======================

The ``dvrk_console`` package provides the ``stereo_display`` executable, which
constructs custom GStreamer topologies from its JSON configuration.  The display
pipeline is the visualization end of the architecture: it renders video for the
surgeon's console and can expose selected streams for other local processes, but
session recording and extraction are handled by
:doc:`/pages/video/software/dvrk/dvrk_data/index`.

Hardware Acceleration
---------------------

Stereo hardware sources bypass standard memory copying entirely whenever possible.

By passing variables like ``preserve_size`` and exact coordinate subsets directly down to standard elements (``videocrop``, ``videoscale``), the pipeline resolves spatial padding locally before issuing payloads up to complex multi-sinks (e.g. ``glimagesink``, OpenGL texture mappers, or generic ``appsink`` drop zones).

Zero-Copy IPC via Unix FD Sinks
-------------------------------

To support modular multi-process architectures without incurring the high CPU
and latency overhead of copying high-definition frames over ROS, the viewer
supports zero-copy video routing using ``unixfdsink`` elements.  This is the
same local video transport mechanism used by ``dvrk_data`` utilities.

When configured via ``unixfdsinks`` in the JSON configuration:

* The viewer exports video frames directly as Unix file descriptors (shared memory buffers) over Unix domain sockets.
* You can route the ``left``, ``right``, ``stereo`` (side-by-side composition), or ``overlay`` (composed with HUD) streams.
* Other local processes, such as custom machine-vision tools or ``dvrk_data``
  recording pipelines, can connect to these Unix sockets to consume video
  streams with near-zero latency.

ROS Integration via gscam
-------------------------

Although ``dvrk_console`` does not publish ROS image topics directly, any active
``@dvrk_gst`` abstract socket can be bridged into ROS image topics using the
``gscam_socket`` helper from ``dvrk_data``.  Sockets are created by
``stereo_source``, ``stereo_alignment``, ``stereo_display``, or any other
application that writes to the abstract Unix-socket transport.

Discovering available sockets
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Run ``gscam_socket`` with no arguments to list every active ``@dvrk_gst`` socket
on the local machine:

.. code-block:: bash

   ros2 run dvrk_data gscam_socket

Example output::

   Available @dvrk_gst sockets:
     [1] @dvrk_gst:stereo_source:left
     [2] @dvrk_gst:stereo_source:right
     [3] @dvrk_gst:stereo_alignment:stereo

   Usage: gscam_socket <socket>
   Example: gscam_socket @dvrk_gst:stereo_source:left

Launching a gscam node
~~~~~~~~~~~~~~~~~~~~~~

Pass the desired socket to ``gscam_socket``; it verifies the socket is live and
then starts a ``gscam_node`` under a namespace derived from the socket's role
and name (``<role>/<name>``):

.. code-block:: bash

   # Short name — uses stereo_source as the default role
   ros2 run dvrk_data gscam_socket left

   # role:name form
   ros2 run dvrk_data gscam_socket stereo_alignment:stereo

   # Fully-qualified
   ros2 run dvrk_data gscam_socket @dvrk_gst:stereo_display:overlay

For ``gscam_socket left`` the image is published at
``/stereo_source/left/image_raw`` and camera info at
``/stereo_source/left/camera_info``.

The namespace, TF ``frame_id``, and ``camera_name`` can be overridden:

.. code-block:: bash

   ros2 run dvrk_data gscam_socket left \
     --namespace left_camera \
     --frame-id left_optical_frame

Configuring unixfdsinks
~~~~~~~~~~~~~~~~~~~~~~~

Any process that exposes abstract sockets will make them available to
``gscam_socket``.  For ``stereo_display``, add ``unixfdsinks`` entries to the
JSON configuration with the stream name identified by a ``"socket"`` key:

.. code-block:: json

   "unixfdsinks": [
     { "socket": "stereo" },
     { "socket": "overlay" }
   ]

For ``stereo_source`` or ``stereo_alignment`` the same ``"socket"`` key is used
in their respective configuration files (see
:doc:`/pages/video/software/dvrk/dvrk_data/index`).

.. note::
   ``gscam_socket`` calls ``ros2 launch dvrk_data gscam.launch.py`` internally.
   The launch file can also be invoked directly with a fully-qualified socket
   name when scripting or integrating with other launch files:

   .. code-block:: bash

      ros2 launch dvrk_data gscam.launch.py socket:=@dvrk_gst:stereo_source:left

Overlay HUD
------------

The viewer renders a real-time heads-up display (HUD) directly onto the video feed using a GStreamer ``cairooverlay`` element (named ``stereo_overlay``). The overlay subscribes to the :ref:`dVRK System Node <system>` ROS topics (via the ``dvrk_console_namespace``) and reflects the current operational state. The overlay opacity is controlled by the ``overlay_alpha`` configuration field.

The following indicators are displayed:

**Foot Pedals (bottom center)**
   Two small status circles appear at the bottom center of each eye. From left to right they represent:

   - **Clutch**: Filled when the clutch foot pedal is actively pressed.
   - **Camera**: Filled when the camera foot pedal is actively pressed.
   - For both pedals, filled green for quick taps.

**PSM Teleop Indicators (bottom corners)**
   Numbered circles showing which PSMs are currently teleoperated:

   - **Left column** (bottom-left corner): PSMs controlled by the MTML (left master).
   - **Right column** (bottom-right corner): PSMs controlled by the MTMR (right master).
   - The circle displays the PSM number (e.g., ``1``, ``2``, ``3``).
   - **Filled** when the teleop ``following`` state is active (the surgeon is controlling the arm).
   - **Outlined** when selected but not currently following.
   - **Red** when the PSM's ``measured_cp`` reports an invalid pose (zero timestamp).
   - The **tool type** label (e.g., "Large Needle Driver") is displayed next to each PSM indicator.

**Camera Teleop (top center)**
   A camera icon appears at the top center when camera (ECM) teleoperation is active:

   - **Filled** when the ECM ``following`` state is active.
   - **Outlined** when the ECM teleop is selected but not following.

Augmented Reality (AR) Overlay Pipeline
----------------------------------------

When AR is enabled (``ar.enabled: true``), the stereo pipeline constructs a dual-layer video composition for each eye using hardware-accelerated ``glvideomixer`` elements (named ``left_ar_mix`` and ``right_ar_mix``).

**Architecture**

* **Layer 1 (Background):** The live left and right endoscope camera streams.
* **Layer 2 (Foreground Overlay):** The AR overlay streams pulled from the UNIX domain sockets via ``unixfdsrc`` elements (``left_ar_src`` and ``right_ar_src``).

**Color Keying (Chroma Keying)**

If a ``color_key`` (RGB) is specified in the configuration, the viewer automatically injects a GStreamer ``alpha`` element set to ``method=custom`` targeting the specified color. This converts matching background pixels in the AR stream to transparent alpha, allowing the background endoscope video to show through.

**Dynamic Frame Rate & Synchronization**

To allow external AR generators (e.g. AI inference or complex calculation scripts) to run at variable frame rates without lagging or freezing the main display, the viewer attaches GStreamer pad probes to the ``src`` pad of the AR sources. These probes rewrite the Presentation Timestamp (PTS) of incoming AR frames using the pipeline's current running time:

* This decouples the display framerate from the AR inference framerate.
* The ``glvideomixer`` will continuously render the latest available AR frame over the live, full-frame-rate background feed.
