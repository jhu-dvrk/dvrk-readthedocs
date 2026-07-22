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

``stereo_source`` owns the left and right camera pipelines.  It converts each
output to I420, places bounded leaky queues before consumers, records a
``CLOCK_REALTIME`` source timestamp in the buffer metadata, and publishes the
two streams on abstract Unix sockets.

.. code-block:: bash

   ros2 run dvrk_data stereo_source -c stereo_source.json

When no matching outputs are listed, it creates the conventional sockets
``@dvrk_gst:stereo_source:left`` and
``@dvrk_gst:stereo_source:right``.

``stereo_alignment``
====================

``stereo_alignment`` consumes the two eyes, applies per-eye color correction,
crop, horizontal shift, and vertical shift, then creates an I420 side-by-side
stereo frame.  It preserves the source observations and adds a
``stereo_output`` timestamp.

.. code-block:: bash

   ros2 run dvrk_data stereo_alignment -c stereo_alignment.json

Its conventional output is
``@dvrk_gst:stereo_alignment:stereo``.  :doc:`/pages/video/software/dvrk/dvrk_console/stereo-display`
and :doc:`record` normally consume this socket independently.

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

Every dVRK GStreamer socket uses the Linux abstract namespace and follows:

.. code-block:: text

   @dvrk_gst:<role>:<name>

Configuration accepts a short name, a role and name, or the fully qualified
form:

.. code-block:: json

   {"socket": "left"}
   {"socket": "stereo_alignment:stereo"}
   {"socket": "@dvrk_gst:stereo_alignment:stereo"}

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

   ros2 run dvrk_data gscam_socket stereo_source:left

The launch file can also be invoked directly when the fully qualified name is
known:

.. code-block:: bash

   ros2 launch dvrk_data gscam.launch.py \
     socket:=@dvrk_gst:stereo_source:left

The launch arguments are:

.. list-table::
   :header-rows: 1

   * - Argument
     - Required
     - Default
   * - ``socket``
     - yes
     - none
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
``/stereo_source/left/image_raw`` and camera information under
``/stereo_source/left/camera_info``.  The bridge uses a one-buffer leaky queue,
``videoconvert``, and ``gscam_node`` with sink synchronization disabled.
Downstream of this bridge, normal ROS image-transport costs apply.

Stereo calibration
******************

The runtime application and calibration script have different roles:

* ``stereo_alignment`` continuously performs the saved transformation.
* ``stereo_alignment_calibration`` interactively finds the crop, horizontal
  and vertical alignment, and color adjustments and writes them to the JSON
  file.

Start the source, calibrate, then run the alignment process:

.. code-block:: bash

   ros2 run dvrk_data stereo_source -c stereo_source.json
   ros2 run dvrk_data stereo_alignment_calibration -c stereo_alignment.json
   ros2 run dvrk_data stereo_alignment -c stereo_alignment.json

The calibration preview overlays green-tinted left and magenta-tinted right
frames.  Use the arrow keys for horizontal and vertical alignment, ``+`` and
``-`` for crop size, ``o`` and ``p`` for crop-width correction, ``c`` to cycle
color matching, ``f`` for fullscreen, and ``q`` to finish and choose whether
to save.  Add ``--split`` to open one preview window per eye.

Stereo alignment makes corresponding image content coincide.  It does not
create hardware camera synchronization; the two source timestamps remain
available so their acquisition separation can be inspected.

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
