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
     "sinks": ["glimage"]
   }

``gst_input`` accepts a canonical dVRK socket reference or a plain GStreamer
source fragment.  ``eye_size`` is one eye's dimensions; the side-by-side input
is twice this width.

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
     - Optional canonical dVRK socket for the composed stereo output.
   * - ``eye_size``
     - Width and height of one eye.
   * - ``overlay_alpha``
     - HUD opacity from 0.0 to 1.0; default 0.7.
   * - ``display_horizontal_offset_px``
     - Working-depth calibration written by ``stereo_display_calibration``.
   * - ``sinks``
     - Any combination of ``glimage`` and ``glimages``.

Example outputs and picture-in-picture
**************************************

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "surgeon_display",
     "dvrk_console_namespace": "console",
     "overlay_alpha": 0.7,
     "display_horizontal_offset_px": 18,
     "gst_input": "@dvrk:stereo_alignment:stereo",
     "gst_output": "@dvrk:stereo_display:stereo",
     "eye_size": {"width": 1920, "height": 1080},
     "sinks": ["glimages"],
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
clamped to 0.01 through 0.99.

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

The current field names are ``left`` and ``right``; each is an endpoint object
with a ``gst_input`` field.

The installed ``stereo_display.schema.json`` is the authoritative structural
reference for these fields.
