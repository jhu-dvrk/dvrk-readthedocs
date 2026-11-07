.. _dvrk-video-configuration:

Video configuration
###################

All video applications require a versioned JSON ``type``.  Socket fields use
the naming rules in :ref:`dvrk-video-data`.  The installed schemas are under
``share/dvrk_data/schemas``.

Stereo source
*************

.. code-block:: json

   {
     "type": "dvrk_data:stereo_source@1.0.0",
     "name": "endoscope",
     "camera": {
       "size": {"width": 1920, "height": 1080},
       "left": {
         "gst_input": "v4l2src device=/dev/video-left ! video/x-raw,width=1920,height=1080,framerate=30/1"
       },
       "right": {
         "gst_input": "v4l2src device=/dev/video-right ! video/x-raw,width=1920,height=1080,framerate=30/1"
       }
     }
   }

The ``gst_input`` values are GStreamer source fragments, not complete
pipelines with sinks.  Add device-specific elements such as ``deinterlace`` to
the fragment when required. The root ``gst_output`` defaults to
``@dvrk:stereo_source:stereo``, a combined LR stream. To enable separate eye
outputs as well, explicitly set ``camera.left.gst_output`` to
``@dvrk:stereo_source:left`` and ``camera.right.gst_output`` to
``@dvrk:stereo_source:right``. An omitted or empty per-eye output is disabled.

``camera.size`` may be ``"auto"`` to detect incoming dimensions, or an object
with the expected per-eye width and height. Source and alignment use the
unconditional :ref:`dvrk-latest-frame-policy`; no synchronization option needs
to be added to these JSON files.

Stereo alignment
****************

.. code-block:: json

   {
     "type": "dvrk_data:stereo_alignment@1.0.0",
     "name": "endoscope",
     "preserve_size": true,
     "gst_input": "@dvrk:stereo_source:stereo",
     "gst_output": "@dvrk:stereo_alignment:stereo",
     "camera": {
       "size": {"width": 1920, "height": 1080},
       "crop": {"width": 1800, "height": 1012},
       "alignment": {
         "horizontal_shift_px": 0,
         "vertical_shift_px": 0
       }
     }
   }

The root ``gst_input`` defaults to ``@dvrk:stereo_source:stereo``. Processing
accepts only a combined side-by-side left/right stream. Use ``-i`` /
``--input`` to override it explicitly; separate eye processing inputs are
unsupported.

Color correction is stored independently in ``stereo_color_adjustment.json``.
Both ``stereo_alignment`` and ``stereo_rectify`` discover that file beside their
processing configuration, or accept ``--color-adjustment /path/to/file.json``.
An absent discovered file means neutral correction. Generate the shared file
with ``ros2 run dvrk_data stereo_find_color_adjustment``. Its type is
``dvrk_data:stereo_color_adjustment@1.0.0`` and its ``left`` and ``right`` objects
contain ``brightness``, ``contrast``, ``saturation``, and ``hue``.

``camera.size`` accepts ``"auto"`` or fixed per-eye dimensions. Crop and shift
values remain input-pixel calibration values and must fit the detected camera
size. The root ``gst_output`` defaults to ``@dvrk:stereo_alignment:stereo``;
set it to an empty string to disable the socket output.

With ``preserve_size`` enabled, the aligned crop is scaled back to the declared
per-eye camera size after maintaining its aspect ratio.  Disable it to publish
the cropped dimensions directly.

Schema authority
****************

The schemas describe the fields read by the current executables; they do not
turn arbitrary GStreamer fragments into portable configurations.  Device
names, plugin availability, caps, and hardware memory features remain specific
to the capture computer.  Validate JSON structure first, then use the pipeline
printed by each executable to diagnose media negotiation.
