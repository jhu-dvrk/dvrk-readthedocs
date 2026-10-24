Frame timestamp metadata
########################

``dvrk_data`` carries ``DataCollectionFrameTimestamps`` metadata with video
buffers over Unix FD sockets. The metadata includes a ``channel`` value
(``left``, ``right``, ``stereo-LR``, or ``stereo-RL``), CPU stage times, and
per-eye reference clock timestamps. Each reference entry preserves the
original ``GstReferenceTimestampMeta`` caps string, timestamp, and duration.
This includes DeckLink hardware and stream time when supplied by the input.
Hardware clock epochs remain device-defined; they are not interchangeable
with CPU time or buffer PTS. Empty lists mean unavailable; timestamp zero is
valid.

CPU stage fields use ``CLOCK_REALTIME`` nanoseconds (zero means unavailable):

* ``left-source-ts`` and ``right-source-ts``: observations in the capture
  branches.
* ``source-stereo-output-ts``: initial side-by-side composition on
  ``@dvrk:stereo_source:stereo``.
* ``alignment-output-ts``: output of the alignment compositor.
* ``overlay-output-ts``: overlay/display output processing.
* ``stereo-output-ts``: retained as the legacy alignment-output alias.

Composition copies metadata from the samples actually selected by the
compositor, including when an eye repeats a frame. Display mixers preserve
camera provenance and exclude annotation/PiP provenance. If a mixer selects
eyes from different stereo pairs, their per-eye timestamps remain available,
but any unequal shared stage timestamps are marked unavailable instead of
attributing one pair's time to both images.

The recorder includes these CPU fields under each frame's ``cpu.realtime`` and
reference entries under ``reference_timestamps.left``, ``.right``, and
``.mono`` in its timestamp JSON. Reference entries contain ``reference``,
``timestamp_ns``, and ``duration_ns``; unavailable timestamp/duration values
are JSON null.

Differences between available CPU stage times estimate processing and queueing
latency on the same host. They exclude camera exposure and display scanout,
and wall-clock adjustments can affect them. Do not subtract hardware/reference
clock timestamps from CPU timestamps without an established clock mapping.

Build with ``BUILD_TESTING=ON`` and run ``ctest`` for CPU composition, metadata
serialization, and Unix FD transport tests. With a working OpenGL display,
``stereo_timestamp_meta_test --gl`` and ``--camera`` additionally exercise
stereo and camera-plus-AR mixer metadata propagation.
