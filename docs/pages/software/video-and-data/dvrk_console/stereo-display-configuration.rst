.. _dvrk-stereo-display-configuration:

Stereo display configuration
############################

The display consumes a side-by-side stereo stream produced by
``dvrk_data stereo_alignment``.  A minimal configuration is:

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "surgeon_display",
     "dvrk_console_namespace": "console",
     "gst_input": "@dvrk:stereo_alignment:stereo",
     "eye_size": {"width": 1920, "height": 1080},
     "sinks": ["side_by_side"]
   }

``gst_input`` accepts a canonical dVRK socket reference or a plain GStreamer
source fragment.  ``eye_size`` is one eye's dimensions; the side-by-side input
is twice this width. Use ``"eye_size": "auto"`` to detect dimensions from
the incoming stereo stream. If ``gst_input`` is omitted, it defaults to
``@dvrk:stereo_alignment:stereo``.

Latest-frame selection is unconditional. The :ref:`dvrk-frame-timing` panel is
always present in the control window; neither feature needs an enable flag.

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
   * - ``gst_input``
     - Main side-by-side input: a canonical ``@dvrk:role:name`` socket or a plain GStreamer source fragment.
   * - ``gst_output``
     - Canonical dVRK socket for the main stereo video and HUD; default ``@dvrk:stereo_display:overlay``. Empty disables the socket. PiP tiles belong to the video-window layouts.
   * - ``eye_size``
     - Width and height of one eye, or ``"auto"`` to detect the incoming size.
   * - ``overlay_alpha``
     - HUD opacity from 0.0 to 1.0; default 0.7.
   * - ``display_horizontal_shift_px``
     - Signed shift of the stereo video on this display, in final per-eye pixels. Positive moves left-eye pixels right and right-eye pixels left. Default 0.
   * - ``widget_horizontal_disparity_px``
     - Signed disparity of fixed HUD widgets and extra-stream tiles, in final per-eye pixels. Positive places them nearer; zero is the display plane. Default 0.
   * - ``display_horizontal_offset_px``
     - Deprecated name for the widget setting with the opposite sign. It cannot be combined with ``widget_horizontal_disparity_px``.
   * - ``sinks``
     - Any combination of ``side_by_side``, ``separate``, and ``headless``.

Example outputs and picture-in-picture
**************************************

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "surgeon_display",
     "dvrk_console_namespace": "console",
     "overlay_alpha": 0.7,
     "display_horizontal_shift_px": 0,
     "widget_horizontal_disparity_px": -18,
     "gst_input": "@dvrk:stereo_alignment:stereo",
     "gst_output": "@dvrk:stereo_display:stereo",
     "eye_size": {"width": 1920, "height": 1080},
     "sinks": ["separate"],
     "pip_gst_inputs": {
       "monos": [
         {"gst_input": "videotestsrc pattern=ball is-live=true ! video/x-raw,width=640,height=480,framerate=10/1"}
       ],
       "scale": 0.3
     }
   }

At most two entries across ``pip_gst_inputs.monos`` and
``pip_gst_inputs.stereos`` are used.  Additional entries are discarded with a
warning.  Each entry contains a ``gst_input`` field, which may be a plain
GStreamer source fragment or a canonical ``@dvrk:role:name`` socket.  Scale is
clamped to 0.01 through 0.99. Extra feeds use the
:ref:`dvrk-latest-frame-policy`; their update rates are independent of the
camera's and a missing frame does not block camera layout updates.

Calibrate camera crop and alignment first, then use the
``stereo_display_calibration.launch.py`` workflow in :ref:`dvrk-stereo-display`
to set video shift and widget depth. The video shifts within each eye, leaving
black pixels at the exposed edge. Touch-panel annotations use final display
coordinates and remain registered after the shift.

If your extra stream come from the SlicerGStreamer plugin (https://github.com/rosmed/SlicerGStreamer), you can use something like:

.. code-block:: json

   {
     "pip_gst_inputs": {
       "monos": [
         {"gst_input": "unixfdsrc socket-path=/tmp/slicer_gstreamer_View1_lconnol8.sock do-timestamp=true ! videoconvert"}
       ]
     }
   }

AR configuration
****************

.. code-block:: json

   {
     "ar": {
       "enabled": true,
       "left": {"gst_input": "@dvrk:stereo_source:left_ar"},
       "right": {"gst_input": "@dvrk:stereo_source:right_ar"},
       "color_key": [0, 255, 0]
     }
   }

``left`` and ``right`` each accept a separate eye endpoint with a
``gst_input`` field. For the combined side-by-side LR annotation stream from
:ref:`dvrk-control-panel`, use ``ar.stereo`` instead:

.. code-block:: json

   {
     "ar": {
       "enabled": true,
       "stereo": {"gst_input": "@dvrk:control_panel:annotations"}
     }
   }

If AR is enabled without another input, the stereo input defaults to this
socket. If the panel overrides ``annotations.gst_output``, set
``ar.stereo.gst_input`` to the same socket name. The stereo annotation stream
uses transparency, so it does not need ``color_key``.

The installed ``stereo_display.schema.json`` is the authoritative structural
reference for these fields.
