Stereo display configuration
############################

The display consumes a side-by-side stereo stream produced by
``dvrk_data stereo_alignment``.  A minimal configuration is:

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "surgeon_display",
     "dvrk_console_namespace": "console",
     "stereo": {
       "socket": "stereo_alignment:stereo",
       "eye_size": {"width": 1920, "height": 1080}
     },
     "sinks": ["glimage"]
   }

``stereo.socket`` accepts the three socket forms described in
:doc:`/pages/video/software/dvrk/dvrk_data/video-data`.  A raw GStreamer source
fragment can instead be supplied as ``stereo.stream``.  ``eye_size`` is one
eye's dimensions; the older ``stereo.size`` form describes the total
side-by-side width.

Top-level fields
****************

.. list-table::
   :header-rows: 1

   * - Field
     - Purpose
   * - ``name``
     - Display instance and persisted-settings name.
   * - ``dvrk_console_namespace``
     - Console namespace used by the HUD; default ``console``.
   * - ``overlay_alpha``
     - HUD opacity from 0.0 to 1.0; default 0.7.
   * - ``display_horizontal_offset_px``
     - Working-depth calibration written by ``stereo_display_calibration``.
   * - ``sinks``
     - Any combination of ``glimage`` and ``glimages``.
   * - ``unixfdsinks``
     - ``stereo`` or ``overlay`` local output sockets.

Example outputs and picture-in-picture
**************************************

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "surgeon_display",
     "dvrk_console_namespace": "console",
     "overlay_alpha": 0.7,
     "display_horizontal_offset_px": 18,
     "stereo": {
       "socket": "stereo_alignment:stereo",
       "eye_size": {"width": 1920, "height": 1080}
     },
     "sinks": ["glimages"],
     "unixfdsinks": [
       {"socket": "stereo"},
       {"socket": "overlay"}
     ],
     "extra_streams": {
       "monos": [
         "videotestsrc pattern=ball is-live=true ! video/x-raw,width=640,height=480,framerate=10/1"
       ],
       "scale": 0.3
     }
   }

At most two entries across ``extra_streams.monos`` and
``extra_streams.stereos`` are used.  Additional entries are discarded with a
warning.  Scale is clamped to 0.01 through 0.99.

AR configuration
****************

.. code-block:: json

   {
     "ar": {
       "enabled": true,
       "left": "stereo_source:left_ar",
       "right": "stereo_source:right_ar",
       "color_key": [0, 255, 0]
     }
   }

The current field names are ``left`` and ``right``.  Older documentation that
uses ``left_socket`` or ``right_socket`` does not match the parser.

The installed ``stereo_config.schema.json`` is the authoritative structural
reference for these fields.
