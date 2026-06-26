Integration & Pipeline
======================

The package provides the ``stereo`` executable (built from ``main_stereo.cpp``), which constructs custom GStreamer topologies from its designated JSON schema.

Hardware Acceleration
---------------------

Stereo hardware sources bypass standard memory copying entirely whenever possible.

By passing variables like ``preserve_size`` and exact coordinate subsets directly down to standard elements (``videocrop``, ``videoscale``), the pipeline resolves spatial padding locally before issuing payloads up to complex multi-sinks (e.g. ``glimagesink``, OpenGL texture mappers, or generic ``appsink`` drop zones).

Zero-Copy IPC via Unix FD Sinks
-------------------------------

To support modular and flexible multi-process architectures without incurring the high CPU and latency overhead of copying high-definition frames over ROS, the viewer supports zero-copy video routing using ``unixfdsink`` elements. 

When configured via ``unixfdsinks`` in the JSON configuration:

* The viewer exports video frames directly as Unix file descriptors (shared memory buffers) over Unix domain sockets.
* You can route the ``left``, ``right``, ``stereo`` (side-by-side composition), or ``overlay`` (composed with HUD) streams.
* Other local processes (such as custom machine-vision or video recording nodes) can connect to these Unix sockets to consume the video streams with near-zero latency.

ROS Integration via gscam
-------------------------

Although ``dvrk_display`` does not publish ROS image topics directly, you can easily bridge any configured Unix FD sink back to ROS using the ``gscam`` package. This is done by creating a ``gscam`` GStreamer pipeline that uses the ``unixfdsrc`` source element.

For example, to publish an uncalibrated stereo pair over ROS alongside the ``dvrk_display stereo`` application:

1. Configure two ``unixfdsinks`` in the JSON configuration file, one for the ``left`` stream and one for the ``right`` stream:

   .. code-block:: json

      "unixfdsinks": [
        { "stream": "left", "name": "left_raw" },
        { "stream": "right", "name": "right_raw" }
      ]

2. Launch ``gscam`` nodes configured to read from these sockets. The corresponding GStreamer pipeline for ``gscam`` would look like:

   .. code-block:: bash

      export GSCAM_CONFIG="unixfdsrc socket-path=/tmp/dvrk_display_left_raw_<user>.sock ! capsfilter caps=\"video/x-raw,format=I420,width=640,height=480,framerate=30/1\" ! videoconvert"
      ros2 run gscam gscam_node --ros-args --remap /camera/image_raw:=/left/image_raw

This approach allows external vision processing, recording, or ROS-based visualization nodes to access the individual streams concurrently without affecting the primary low-latency surgeon's display.

.. note::
   You will likely need to change the gstreamer caps for ``gscam`` to match the actual format of the video stream.

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
