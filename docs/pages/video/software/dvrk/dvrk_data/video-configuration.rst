Video configuration
###################

All video applications require a versioned JSON ``type``.  Socket fields use
the naming rules in :doc:`video-data`.  The installed schemas are under
``share/dvrk_data/schemas``.

Stereo source
*************

.. code-block:: json

   {
     "type": "dvrk_data:stereo_source@1.0.0",
     "name": "endoscope",
     "unixfdsinks": [
       {"socket": "left"},
       {"socket": "right"}
     ],
     "camera": {
       "size": {"width": 1920, "height": 1080},
       "left": {
         "stream": "v4l2src device=/dev/video-left ! video/x-raw,width=1920,height=1080,framerate=30/1"
       },
       "right": {
         "stream": "v4l2src device=/dev/video-right ! video/x-raw,width=1920,height=1080,framerate=30/1"
       }
     }
   }

The ``stream`` values are GStreamer source fragments, not complete pipelines
with sinks.  Add device-specific elements such as ``deinterlace`` to the
fragment when required.

Stereo alignment
****************

.. code-block:: json

   {
     "type": "dvrk_data:stereo_alignment@1.0.0",
     "name": "endoscope",
     "preserve_size": true,
     "unixfdsources": [
       {"socket": "left"},
       {"socket": "right"}
     ],
     "unixfdsinks": [
       {"socket": "stereo"}
     ],
     "camera": {
       "size": {"width": 1920, "height": 1080},
       "left": {
         "color": {
           "brightness": 0.0,
           "contrast": 1.0,
           "saturation": 1.0,
           "hue": 0.0
         }
       },
       "right": {
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

If ``camera.left.stream`` and ``camera.right.stream`` are absent,
``stereo_alignment`` resolves the left and right entries in
``unixfdsources`` against the ``stereo_source`` role.  Direct GStreamer source
fragments can be supplied instead for a single-process acquisition/alignment
pipeline.

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
