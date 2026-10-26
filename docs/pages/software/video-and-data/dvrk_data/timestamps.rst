.. _dvrk-timestamps:

Video timestamps
################

The dVRK video path preserves several timing observations instead of replacing
them with a single timestamp whose meaning is ambiguous.  Values transported
between local processes are stored in custom GStreamer buffer metadata.

CPU realtime observations
*************************

The following fields use ``CLOCK_REALTIME`` and nanoseconds since the Unix
epoch:

.. list-table::
   :header-rows: 1

   * - Field
     - Observation
   * - ``mono_source_ns``
     - A monoscopic source stage produced the frame.
   * - ``left_source_ns``
     - ``stereo_source`` observed the left frame.
   * - ``right_source_ns``
     - ``stereo_source`` observed the right frame.
   * - ``source_stereo_output_ns``
     - ``stereo_source`` published the selected side-by-side image.
   * - ``alignment_output_ns``
     - ``stereo_alignment`` produced the aligned side-by-side image.
   * - ``stereo_output_ns``
     - Legacy alias for ``alignment_output_ns``.
   * - ``overlay_output_ns``
     - The display overlay output was produced.
   * - ``recorder_reception_ns``
     - The buffer reached the recorder timestamp probe.

These fields appear under ``frames[].cpu.realtime`` in recording sidecars.
``CLOCK_REALTIME`` is wall-clock time and can be adjusted; it is not a monotonic
clock or an observation of camera exposure. The left and right values are
independent. Their difference reveals software arrival separation without
establishing hardware trigger synchronization.

The :ref:`dvrk-latest-frame-policy` preserves the source observations attached
to the selected pixels, even when one eye is reused. A shared upstream stage
observation is available only when both selected eyes came from the same
upstream pair. Otherwise that stage time is zero in transport metadata and
``null`` in the recording sidecar. A newly composed output can therefore have
a new alignment time while one of its source images is old. Inspect per-eye
source times rather than using the newest stage time as evidence of fresh
camera video.

The display's :ref:`dvrk-frame-timing` panel uses these CPU source observations
for its ages and channel difference. It does not include latency before the
source probe or physical monitor scanout.

Hardware and reference-clock observations
*****************************************

Available reference timestamps remain attached to the selected eye images and
are recorded under ``reference_timestamps.mono``, ``reference_timestamps.left``,
and ``reference_timestamps.right``. Each entry retains its reference-domain
identifier, timestamp, and duration. Missing reference timestamps remain
unavailable; a CPU observation is not substituted into a hardware clock domain.

Separate capture devices may have independent reference-clock epochs. Do not
subtract hardware times across devices without establishing a common clock.

GStreamer timing
****************

Each video sidecar also preserves buffer PTS, DTS, duration, running time,
stream time, and a sample of the pipeline-selected GStreamer clock.  These
values belong to the clock domains declared in the sidecar:

* PTS locates the presentation frame in the encoded media timeline.
* Running time locates a buffer relative to its pipeline's clock/base time.
* Stream time describes position in the media stream.
* GStreamer clock time is not assumed to use the Unix epoch.

The live spatial mixers select each input's newest image by arrival, without
matching input PTS. Their new output PTS describe publication in the output
pipeline; they do not replace the original per-eye source observations.
Variable-rate output may have no fixed buffer duration. Encoding or other
GStreamer consumers can still use their own media-timeline scheduling.

Annotation timestamp
********************

Downstream tools need one Unix-epoch value for range selection, output names,
and tag conversion.  The sidecar does not store a derived timestamp; each tool
selects an annotation timestamp from the explicit observations in the frame.
The current preference order is:

#. mono source
#. midpoint of left and right source observations
#. left source
#. right source
#. recorder reception

``timestamp_statistics`` counts frames that carried metadata through
``unixfd`` and frames that fell back to reception time.  A fallback remains a
valid observation but includes all latency before the recorder.

Recording and extraction
************************

The recorder writes one ``dvrk_data:video_sidecar@1.0.0`` JSON file next to
each MP4 segment.  :ref:`dvrk-extract`, :ref:`dvrk-tagging`, and ``encord_to_tags`` use
the annotation timestamp for correlation and output names.  Extraction uses
the recorded ``frame_index`` to locate frames in the encoded video.  This
separation avoids treating a Unix timestamp as a media seek position.

``estimated_latency`` is a separate empirical correction.  It is stored in a
record configuration in seconds and copied to the sidecar in milliseconds.
It does not alter the original observations, is not applied by extraction,
and does not natively synchronize independent sources.
