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
         "gst_input": "v4l2src device=/dev/video-left ! video/x-raw,width=1920,height=1080,framerate=30/1",
         "gst_output": "@dvrk:stereo_source:left"
       },
       "right": {
         "gst_input": "v4l2src device=/dev/video-right ! video/x-raw,width=1920,height=1080,framerate=30/1",
         "gst_output": "@dvrk:stereo_source:right"
       }
     }
   }

The ``gst_input`` values are GStreamer source fragments, not complete
pipelines with sinks.  Add device-specific elements such as ``deinterlace`` to
the fragment when required.  ``gst_output`` is optional; when omitted,
``stereo_source`` publishes the conventional ``@dvrk:stereo_source:left`` and
``@dvrk:stereo_source:right`` sockets.

Stereo alignment
****************

.. code-block:: json

   {
     "type": "dvrk_data:stereo_alignment@1.0.0",
     "name": "endoscope",
     "preserve_size": true,
     "gst_output": "@dvrk:stereo_alignment:stereo",
     "camera": {
       "size": {"width": 1920, "height": 1080},
       "left": {
         "gst_input": "@dvrk:stereo_source:left",
         "color": {
           "brightness": 0.0,
           "contrast": 1.0,
           "saturation": 1.0,
           "hue": 0.0
         }
       },
       "right": {
         "gst_input": "@dvrk:stereo_source:right",
         "color": {
           "brightness": 0.0,
           "contrast": 1.0,
           "saturation": 1.0,
           "hue": 0.0
         }
       },
       "crop": {"width": 1800, "height": 1012},
       "alignment": {
         "horizontal_shift_px": 0,
         "vertical_shift_px": 0
       }
     }
   }

The ``camera.left.gst_input`` and ``camera.right.gst_input`` values can be
canonical dVRK socket references or direct GStreamer source fragments.
``gst_output`` is optional; when omitted, ``stereo_alignment`` publishes
``@dvrk:stereo_alignment:stereo``.

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
