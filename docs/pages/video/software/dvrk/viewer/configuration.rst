Configuration Schema
====================

The ``dvrk_console stereo_display`` executable uses a JSON configuration file
validated against the ``dvrk_console:stereo_display@1.0.0`` schema.  This
configuration describes the surgeon-console display: camera input streams,
stereo alignment, display sinks, HUD overlays, picture-in-picture streams,
optional AR overlays, and optional ``unixfd`` outputs for other local processes.

Getting Started
---------------

To create a new configuration, start with a minimal JSON file containing only
the essential display fields.

For **Stereo Viewer** (uses ``dvrk_console:stereo_display@1.0.0``):

The ``camera.size``, ``camera.left.stream``, and ``camera.right.stream`` fields are required:

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "dvrk_console",
     "sinks": ["glimage"],
     "camera": {
       "size": { "width": 640, "height": 480 },
       "left":  { "stream": "v4l2src device=/dev/video0" },
       "right": { "stream": "v4l2src device=/dev/video1" }
     }
   }

Most fields (crop dimensions, alignment shifts, color calibration, display offset, etc.) are best adjusted interactively using the :doc:`calibration_tool` rather than edited by hand. The calibration tool will populate and update these fields in the JSON file automatically.

Full Configuration Reference
----------------------------

**Stereo Configuration Example:**

.. code-block:: json

   {
     "type": "dvrk_console:stereo_display@1.0.0",
     "name": "dvrk_console",
     "dvrk_console_namespace": "console",
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
     - **Required.** Must be ``"dvrk_console:stereo_display@1.0.0"``. Validated on startup.
   * - ``name``
     - string
     - Name of the viewer instance used as the ROS node name.
   * - ``dvrk_console_namespace``
     - string
     - ROS namespace of the dVRK console, used to discover teleop and PSM topics for
       the HUD overlay. Default: ``"console"``.
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
       the stereo display presents content at the surgeon's working depth. Set by the
       calibration tool (``[`` / ``]`` keys). Default: ``0``.
   * - ``sinks``
     - array
     - Display output sinks. ``"glimage"`` opens a single window (side-by-side stereo);
       ``"glimages"`` opens two separate per-eye windows. Default: ``[]``.
   * - ``unixfdsinks``
     - array
     - Zero-copy shared-memory outputs via Unix file-descriptor sockets. Each entry is
       an object with a mandatory ``"stream"`` key, an optional ``"name"`` label, and an optional
       ``"socket_path"`` (auto-generated from ``name`` and username when omitted).
       
       * **Stereo streams:** ``"left"``, ``"right"``, ``"stereo"``, or ``"overlay"``.
   * - ``extra_streams``
     - object
     - Optional picture-in-picture streams composited into the bottom of each eye.
       See `Extra Streams`_ below.
   * - ``ar``
     - object
     - Optional Augmented Reality (AR) overlay stream configuration.
       See `Augmented Reality (AR)`_ below.


``camera`` Object
~~~~~~~~~~~~~~~~~

The required ``camera`` object groups all camera-related settings.

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

Augmented Reality (AR)
~~~~~~~~~~~~~~~~~~~~~~

The optional ``ar`` object configures stereoscopic Augmented Reality overlays. The viewer receives graphic frames (with alpha transparency support) via UNIX domain sockets and overlays them on top of the live background cameras:

.. list-table::
   :header-rows: 1
   :widths: 25 10 65

   * - Field
     - Type
     - Description
   * - ``ar.enabled``
     - boolean
     - Enables or disables the AR blending pipeline. Default: ``false``.
   * - ``ar.left_socket``
     - string
     - Path to the UNIX domain socket supplying the left eye AR overlay stream.
   * - ``ar.right_socket``
     - string
     - Path to the UNIX domain socket supplying the right eye AR overlay stream.
   * - ``ar.color_key``
     - array of integers
     - Optional RGB color key (e.g. ``[0, 255, 0]`` for green) to mask out from the overlay stream and turn transparent. When provided, a GStreamer ``alpha`` element is automatically injected.

Geometric Scaling & Size Preservation
-------------------------------------

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

Color correction is applied via ``videobalance`` GStreamer elements. This uses ``camera.left.color`` and ``camera.right.color`` per eye.

Adjusting ``brightness``, ``contrast``, ``saturation``, and ``hue`` corrects manufacturing differences or lighting imbalances. Use the :doc:`calibration_tool` ``c`` key to cycle through automatic matching modes.

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
- ``stereo_config.schema.json`` — Formal JSON Schema for offline stereo validation.
