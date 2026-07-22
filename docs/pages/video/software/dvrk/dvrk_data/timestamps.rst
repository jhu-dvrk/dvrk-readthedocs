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
   * - ``stereo_output_ns``
     - The aligned side-by-side frame left the stereo stage.
   * - ``overlay_output_ns``
     - The display overlay output was produced.
   * - ``recorder_reception_ns``
     - The buffer reached the recorder timestamp probe.

The left and right values are independent.  Their difference can reveal
software or camera arrival skew, but it does not establish hardware trigger
synchronization.

GStreamer timing
****************

Each video sidecar also preserves buffer PTS, DTS, duration, running time,
stream time, and a sample of the pipeline-selected GStreamer clock.  These
values belong to the clock domains declared in the sidecar:

* PTS locates the presentation frame in the encoded media timeline.
* Running time is used for synchronization within a GStreamer pipeline.
* Stream time describes position in the media stream.
* GStreamer clock time is not assumed to use the Unix epoch.

Derived capture time
********************

Downstream tools need one Unix-epoch value for correlation.  The recorder
therefore writes ``derived.preferred_capture_time_ns`` while retaining every
original observation.  The current preference order is:

#. overlay output
#. stereo output
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
each MP4 segment.  :doc:`extract` uses preferred capture time for correlation
and output names, but uses GStreamer PTS to locate the encoded frame.  This
separation avoids treating a Unix timestamp as a media seek position.

``estimated_latency`` is a separate empirical correction.  It is stored in a
record configuration in seconds, copied to the sidecar in milliseconds, and
subtracted by extraction when correlating acquisition ranges.  It does not
alter the original observations or natively synchronize independent sources.
