Configuration Schema
====================

The ``dvrk_display`` stereo executable uses a JSON configuration file validated
against the ``dd::display_config@1.0.0`` schema.

Getting Started
---------------

To create a new configuration, start with a minimal JSON file containing only
the essential fields.  The ``camera.left.stream``, and
``camera.right.stream`` keys are required:

.. code-block:: json

   {
     "type": "dd::display_config@1.0.0",
     "name": "dvrk_display",
     "sinks": ["glimage"],
     "camera": {
       "left":  { "stream": "v4l2src device=/dev/video0" },
       "right": { "stream": "v4l2src device=/dev/video1" }
     }
   }

Most other fields (crop dimensions, alignment shifts, color calibration, display
offset, etc.) are best adjusted interactively using the :doc:`calibration_tool`
rather than edited by hand.  The calibration tool will populate and update the
remaining fields in the JSON file automatically.

Full Configuration Reference
----------------------------

After tuning with the calibration tool, a complete configuration file will look
like:

.. code-block:: json

   {
     "type": "dd::display_config@1.0.0",
     "name": "dvrk_display",
     "dvrk_console_namespace": "console",
     "ros_image_publishers": ["stereo"],
     "overlay_alpha": 0.7,
     "preserve_size": true,
     "display_horizontal_offset_px": 18,
     "sinks": ["glimage"],
     "unixfdsinks": [
       { "stream": "stereo" }
     ],
     "camera": {
       "size": { "width": 640, "height": 480 },
       "left": {
         "stream": "v4l2src device=/dev/video_v4l2_left",
         "color": { "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0 }
       },
       "right": {
         "stream": "v4l2src device=/dev/video_v4l2_right ! videoflip method=rotate-180",
         "color": { "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0 }
       },
       "crop": { "width": 508, "height": 381 },
       "alignment": { "horizontal_shift_px": -89, "vertical_shift_px": 0 }
     }
   }

Top-Level Field Reference
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Field
     - Type
     - Description
   * - ``type``
     - string
     - **Required.** Must be ``"dd::display_config@1.0.0"``.  Validated on startup.
   * - ``name``
     - string
     - Name of the viewer instance used as the ROS node name. Default: ``"dvrk_display"``.
   * - ``dvrk_console_namespace``
     - string
     - ROS namespace of the dVRK console, used to discover teleop and PSM topics for
       the HUD overlay. Default: ``"console"``.
   * - ``ros_image_publishers``
     - array
     - List of ROS image streams to publish. Accepted values: ``"left"``, ``"right"``,
       ``"stereo"``. Default: ``[]`` (none).
   * - ``overlay_alpha``
     - number
     - Opacity of the HUD overlay icons (0.0 = transparent, 1.0 = opaque). Default: ``0.7``.
   * - ``preserve_size``
     - boolean
     - When ``true``, upscales the cropped image back to the original frame dimensions
       without black borders. Default: ``true``.
   * - ``display_horizontal_offset_px``
     - integer
     - Horizontal pixel offset applied symmetrically to each eye's crop window so that
       the stereo display presents content at the surgeon's working depth.  Set by the
       calibration tool (``[`` / ``]`` keys). Default: ``0``.
   * - ``sinks``
     - array
     - Display output sinks. ``"glimage"`` opens a single side-by-side stereo window;
       ``"glimages"`` opens two separate per-eye windows. Default: ``[]``.
   * - ``unixfdsinks``
     - array
     - Zero-copy shared-memory outputs via Unix file-descriptor sockets.  Each entry is
       an object with a mandatory ``"stream"`` key (``"left"``, ``"right"``, ``"stereo"``,
       or ``"overlay"``), an optional ``"name"`` label, and an optional ``"socket_path"``
       (auto-generated from ``name`` and username when omitted).
   * - ``unixfd_socket_path``
     - string
     - *Deprecated shorthand.*  Sets a single stereo unixfd socket path.  Prefer
       ``unixfdsinks`` for new configurations.
   * - ``extra_streams``
     - object
     - Optional picture-in-picture streams composited into the bottom of each eye.
       See `Extra Streams`_ below.

``camera`` Object
~~~~~~~~~~~~~~~~~

The required ``camera`` object groups all camera-related settings:

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Field
     - Type
     - Description
   * - ``camera.size.width`` / ``camera.size.height``
     - integer
     - **Required.** Native frame dimensions of each camera in pixels.
   * - ``camera.left.stream``
     - string
     - **Required.** GStreamer pipeline fragment for the left camera.  Must produce a
       raw video pad at the resolution declared in ``camera.size``.
   * - ``camera.right.stream``
     - string
     - **Required.** GStreamer pipeline fragment for the right camera.  Must match left
       dimensions and frame rate.
   * - ``camera.left.color`` / ``camera.right.color``
     - object
     - Per-eye color correction applied via a ``videobalance`` GStreamer element.  Fields:
       ``brightness`` (−1.0–1.0, default 0.0), ``contrast`` (0.0–2.0, default 1.0),
       ``saturation`` (0.0–2.0, default 1.0), ``hue`` (−1.0–1.0, default 0.0).
       Best set interactively with the :doc:`calibration_tool`.
   * - ``camera.crop.width`` / ``camera.crop.height``
     - integer
     - Size of the cropped eye region used for alignment.  Defaults to ``camera.size``
       when omitted.  Set by the calibration tool (``+`` / ``-`` keys).
   * - ``camera.alignment.horizontal_shift_px``
     - integer
     - Horizontal baseline shift in pixels applied symmetrically in opposite directions
       to the left and right crop windows to align the stereo pair.  Positive values
       converge the images; negative values diverge them.  Set by the calibration tool
       (← / → arrow keys). Default: ``0``.
   * - ``camera.alignment.vertical_shift_px``
     - integer
     - Vertical shift in pixels applied symmetrically to correct vertical misalignment
       between the two cameras.  Set by the calibration tool (↑ / ↓ arrow keys).
       Default: ``0``.

Extra Streams
~~~~~~~~~~~~~

The optional ``extra_streams`` object composites additional video sources as a
picture-in-picture strip at the bottom of each eye.  At most two extra streams
(mono or stereo pairs) are supported:

.. list-table::
   :header-rows: 1
   :widths: 25 10 65

   * - Field
     - Type
     - Description
   * - ``extra_streams.monos``
     - array of strings
     - GStreamer pipeline fragments for additional mono sources.  Each stream is
       displayed in both the left and right eye views.
   * - ``extra_streams.stereos``
     - array of objects
     - Additional stereo pairs, each with a ``"left"`` and ``"right"`` GStreamer
       pipeline fragment.
   * - ``extra_streams.scale``
     - number
     - Fraction of eye height reserved for the extra-stream strip (0.01–0.99).
       Default: ``0.3`` (30 %).

Geometric Scaling & Size Preservation
--------------------------------------

GStreamer crops each eye by computing asymmetric left/right and top/bottom crop
values from ``camera.alignment.horizontal_shift_px``,
``camera.alignment.vertical_shift_px``, and ``camera.crop``.

When ``preserve_size`` is ``true``, an additional aspect-ratio crop is applied
before ``videoscale`` upscales the result back to ``camera.size``.  This
eliminates black borders without distorting the image.

The ``display_horizontal_offset_px`` is folded into the same ``videocrop``
element: the left eye's crop window shifts left and the right eye's shifts right
by the same total offset, so the mixer remains a plain side-by-side compositor
with no ``xpos`` adjustments.

Color Calibration
------------------

``camera.left.color`` and ``camera.right.color`` inject ``videobalance``
GStreamer elements per eye.  Adjusting ``brightness``, ``contrast``,
``saturation``, and ``hue`` corrects manufacturing differences or lighting
imbalances between the two lenses.  Use the :doc:`calibration_tool` ``c`` key to
cycle through automatic matching modes.

Example Configurations
-----------------------

The package ships ready-to-use configuration files under ``share/``:

- ``stereo_viewer.json`` — GStreamer test sources with a unixfd stereo socket.
- ``stereo_v4l2_gooviz.json`` — V4L2 cameras with color calibration and a
  display offset for Gooviz displays.
- ``decklink_gooviz.json`` — Blackmagic Decklink capture cards with Gooviz
  per-eye displays.
- ``doc_stereo_simple.json`` / ``doc_stereo_complex.json`` — Minimal and
  feature-complete documentation templates.
- ``stereo_config.schema.json`` — Formal JSON Schema for offline validation.
