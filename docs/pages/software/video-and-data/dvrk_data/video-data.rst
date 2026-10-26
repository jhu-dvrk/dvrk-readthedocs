.. _dvrk-video-data:

Video data architecture
#######################

The preferred architecture uses several small processes rather than one ROS 2
node that captures, transforms, publishes, displays, and records every image.
Each process owns one part of the media pipeline and exchanges video through
GStreamer's ``unixfdsink`` and ``unixfdsrc`` elements.

Processes and data flow
***********************

``stereo_source``
=================

``stereo_source`` owns the left and right camera pipelines. It converts each
output to I420, records a CPU ``CLOCK_REALTIME`` source timestamp in the buffer
metadata, and combines the latest available eye images into a side-by-side LR
stream. Bounded leaky queues discard superseded frames.

.. code-block:: bash

   ros2 run dvrk_data stereo_source -c stereo_source.json

The root ``gst_output`` defaults to ``@dvrk:stereo_source:stereo``. Separate
``@dvrk:stereo_source:left`` and ``@dvrk:stereo_source:right`` outputs are
optional: set ``camera.left.gst_output`` and ``camera.right.gst_output`` to
publish them. Omitting or emptying either eye's output disables that separate
socket.

``stereo_alignment``
====================

``stereo_alignment`` normally consumes
``@dvrk:stereo_source:stereo``. It applies per-eye color correction, crop,
horizontal and vertical shifts, and optional scaling, then combines the latest
processed eye images into an I420 side-by-side stream. It skips reprocessing an
eye when only its peer's source timestamp changed. It preserves per-eye source
observations and adds an ``alignment_output`` timestamp, also available under
the legacy ``stereo_output`` name.

Separate left and right inputs can be configured instead of the combined
input; see :ref:`dvrk-video-configuration`.

.. code-block:: bash

   ros2 run dvrk_data stereo_alignment -c stereo_alignment.json

Its conventional output is
``@dvrk:stereo_alignment:stereo``.  :ref:`dvrk-stereo-display`
and :ref:`dvrk-record` normally consume this socket independently.

.. _dvrk-latest-frame-policy:

Latest available frames
***********************

``stereo_source``, ``stereo_alignment``, and the display's spatial mixers each
retain one cached image per input. When any input changes, they compose output
from the newest cached images. They do not wait for matching left/right PTS,
matching frame numbers, or an update from the other eye. This is the normal
behavior; there is no JSON synchronization switch.

During streaming, an eye with no image yet is black. If an eye stops updating,
its last image remains while the other continues. If both eyes stop, the display
can retain an old image; :ref:`dvrk-frame-timing` continues aging that image and
marks the unchanged source as stale. Startup dimension detection and connection
recovery still apply; this policy describes frame selection in a running
pipeline.

Combined stereo output is variable-rate. Two independent 30 Hz eye inputs can
produce approximately 60 stereo updates per second, because either eye can
trigger output. This does not increase either camera's acquisition rate. Updates
may be coalesced or dropped when processing cannot keep up. Consumers should
use buffer timestamps rather than assuming a fixed stereo update interval.

Original CPU source times and available reference-clock metadata identify the
images actually selected. New output PTS describe publication time. Preserving
those source observations does not pair images or synchronize hardware cameras;
see :ref:`dvrk-timestamps`.

Why video is not transported as ROS images
*******************************************

A ROS image path represents every frame as a middleware message.  Depending on
the ROS distribution, RMW implementation, publisher/subscriber arrangement,
and loaned-message support, a large stereo frame can require allocation,
serialization, middleware queueing, and one or more copies for each consumer.
Those costs repeat image by image.

GStreamer keeps the data in a media pipeline.  ``unixfdsink`` can send file
descriptors for shareable buffers to another local process, avoiding conversion
of every frame into a ROS message.  GStreamer also provides caps negotiation,
hardware-buffer support, frame dropping, compositing, encoding, and sink
scheduling in the same pipeline model.  Small bounded, leaky queues prevent a
slow display or recorder from accumulating an increasingly stale backlog.

This is not a promise of unconditional end-to-end zero-copy operation.
``videoconvert``, ``videoscale``, encoding, ``gldownload``, the selected camera
driver, or an incompatible allocator can copy or transform a buffer.  The
important architectural property is that local video is not serialized and
transported through ROS 2 by default.

ROS 2 remains the correct path for robot state, commands, status, and
distributed communication.  Use the bridge below when a ROS image interface is
actually required.

Socket names and discovery
**************************

Every dVRK-managed GStreamer socket uses the Linux abstract namespace and
follows:

.. code-block:: text

   @dvrk:<role>:<name>

Configuration fields named ``gst_input`` and ``gst_output`` use the canonical
form for dVRK-managed sockets:

.. code-block:: json

   {"gst_input": "@dvrk:stereo_source:left"}
   {"gst_output": "@dvrk:stereo_alignment:stereo"}

Short names and unprefixed ``role:name`` values are not dVRK socket
references.  A plain GStreamer pipeline may still be supplied in a
``gst_input`` field when no dVRK socket is involved.

Abstract sockets do not create files under ``/tmp`` and are reclaimed by the
kernel when their file descriptors close.  They work only between processes on
the same Linux computer.

List active dVRK sockets with:

.. code-block:: bash

   ros2 run dvrk_data gscam_socket

Bridging a socket to ROS images
*******************************

``gscam_socket`` validates a selected socket and delegates to
``gscam.launch.py``:

.. code-block:: bash

   ros2 run dvrk_data gscam_socket @dvrk:stereo_source:stereo

The launch file can also be invoked directly when the fully qualified name is
known:

.. code-block:: bash

   ros2 launch dvrk_data gscam.launch.py \
     socket:=@dvrk:stereo_source:stereo

If ``socket`` is omitted, the launch file lists currently active ``@dvrk``
sockets and exits without launching ``gscam_node``.

The launch arguments are:

.. list-table::
   :header-rows: 1

   * - Argument
     - Required
     - Default
   * - ``socket``
     - no
     - empty (list active sockets and exit) or ``@dvrk:<role>:<name>``
   * - ``namespace``
     - no
     - ``<role>/<name>``
   * - ``frame_id``
     - no
     - ``<role>_<name>_frame``
   * - ``camera_name``
     - no
     - ``<role>_<name>``

For the example above, images are published under
``/stereo_source/stereo/image_raw`` and camera information under
``/stereo_source/stereo/camera_info``. To bridge only one eye, first enable its
separate source output and select that socket instead.  The bridge uses a one-buffer leaky queue,
``videoconvert``, and ``gscam_node`` with sink synchronization disabled.
Downstream of this bridge, normal ROS image-transport costs apply.

Stereo calibration
******************

Run from the directory containing ``stereo_source.json`` and, if it already
exists, ``stereo_alignment.json``:

.. code-block:: bash

   ros2 launch dvrk_data stereo_alignment_calibration.launch.py

The launch starts ``stereo_source`` and opens the GTK calibration tool after
the source is ready. The tool can create ``stereo_alignment.json`` on its first
run. It previews the stereo socket, with a side-by-side window or separate
left and right windows, plus a difference overlay. Adjust horizontal and
vertical alignment, crop, and optional color correction, then select **Save &
quit**. The launch stops the source when the tool closes. **Quit** asks about
unsaved changes.

**Apply crop** and **Auto-crop for current alignment** start enabled. Auto-crop
finds the largest crop that accommodates the current shifts. The overlay's
brightness and difference-gain sliders affect only the preview; they are not
saved. For another directory, add ``config_dir:=/path/to/configs``. Use
``source_config:=...`` or ``alignment_config:=...`` to override either file
name.

To use the saved calibration for normal operation, launch the complete video
pipeline described in :ref:`dvrk-stereo-display`, which starts source,
alignment, and display in order. The alignment calibration tool previews the
source socket directly; it does not run the separate ``stereo_alignment``
process. Display calibration and the complete video launch include that
process. Use the display timing panel to inspect the full running path. Camera
alignment changes geometry and color; it does not synchronize the hardware
cameras or match timestamps between eyes.

.. _dvrk-gstreamer-dot-files:

Creating GStreamer dot files
****************************

The dVRK video applications can write full-detail GStreamer pipeline graphs
without application-specific command-line options.  Create an output
directory, set GStreamer's standard ``GST_DEBUG_DUMP_DOT_DIR`` environment
variable, and start the application from the same shell:

.. code-block:: bash

   mkdir -p /tmp/dvrk-gst-dot
   GST_DEBUG_DUMP_DOT_DIR=/tmp/dvrk-gst-dot \
     ros2 run dvrk_data stereo_alignment -c stereo_alignment.json

The snapshot is requested after preroll or shortly after a live pipeline
reaches ``PLAYING``.  Its pad labels therefore include negotiated caps, which
are often the most useful information when diagnosing conversions or failed
links.  Filenames include a timestamp and use the application name.

The mechanism is available for ``stereo_source``, ``stereo_alignment``,
``record``, ``dvrk_console/stereo_display``, and the control panel's embedded
video pipeline.  ``record`` creates one graph for each video pipeline and one
for audio when audio is enabled.

Render a generated dot file with Graphviz, for example:

.. code-block:: bash

   dot -Tsvg /tmp/dvrk-gst-dot/<pipeline>.dot -o pipeline.svg

Unset ``GST_DEBUG_DUMP_DOT_DIR`` to disable graph generation.  The variable
must be set before starting the application.
