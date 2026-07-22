Tagging and curation
####################

``video_tag`` reviews one recorded video and creates or updates frame-accurate
stage ranges and event tags.  The selected video is the temporal reference for
the session.

.. code-block:: bash

   ros2 run dvrk_data video_tag -v <session>/<video>.mp4 -c record.json

Optional arguments are ``-t <tags.json>`` for an explicit tag file and ``-T``
to load session tags.  The video is required; the configuration is optional.

Extraction currently supports ``dvrk_data:video_tags@1.0.0`` ranges.  Pass the
file explicitly:

.. code-block:: bash

   ros2 run dvrk_data extract -d <session> -t <video-tags.json>

Encord conversion
*****************

``encord_to_tags`` converts an Encord classification export into the native
video-tag format using a dVRK video sidecar to map frame ranges to preferred
capture timestamps:

.. code-block:: bash

   ros2 run dvrk_data encord_to_tags \
     --encord labels.json \
     --sidecar <session>/<video>.json \
     --output <session>/<video>_tags.json

When ``--output`` is omitted, the script derives a name beside the sidecar.
