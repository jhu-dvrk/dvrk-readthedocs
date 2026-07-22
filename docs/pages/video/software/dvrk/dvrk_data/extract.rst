Extract
#######

``extract`` converts a recorded session into timestamped media and per-topic
CSV files.  It requires either full-session extraction or a supported video
tag file.

Extract the complete session as MP4:

.. code-block:: bash

   ros2 run dvrk_data extract -d <session-directory> --all

Extract timestamped PNG and JPEG frames:

.. code-block:: bash

   ros2 run dvrk_data extract -d <session-directory> --all \
     --format png --format jpg

Options
*******

.. list-table::
   :header-rows: 1

   * - Option
     - Meaning
   * - ``-d``, ``--directory``
     - Session directory containing ``index.json``.
   * - ``-a``, ``--all``
     - Extract one ``full_session`` target.
   * - ``-t``, ``--tags``
     - Extract each range in a ``dvrk_data:video_tags@1.0.0`` file.
   * - ``-f``, ``--format``
     - Add ``jpg``, ``png``, or ``mp4`` output; repeat for multiple formats.
   * - ``-j``, ``--jobs``
     - Number of parallel frame-extraction workers.
   * - ``-S``, ``--split-stereo``
     - Split streams declared as ``LR`` or ``RL`` into left and right outputs.

The default output format is MP4.  Without ``--jobs``, extraction uses half of
the detected CPU count.

How frames are correlated
*************************

The sidecar provides two complementary values.  Preferred capture time is the
Unix-epoch observation used for range selection and output names.  GStreamer
PTS is converted to an encoded-video frame index for seeking.  The configured
latency is subtracted when comparing a stream with a requested acquisition
range.

ROS bag messages in the same range are flattened by topic and written to CSV
with their original bag timestamps.  Tagged extraction creates a numbered
directory for every occurrence of a stage name.

.. note::

   ``dvrk_data:session_tags@1.0.0`` is recognized but is not currently
   supported by extraction.  Use video tags as the extraction reference.
