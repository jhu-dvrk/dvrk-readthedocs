Configuration Schema
====================

The ``dvrk_display`` executables use a JSON configuration file validated against a specific schema. The ``stereo`` executable uses the ``dd::display_config@1.0.0`` schema, while the ``mono`` executable uses the ``dd::mono_config@1.0.0`` schema.

Getting Started
---------------

To create a new configuration, start with a minimal JSON file containing only the essential fields.

For **Stereo Viewer** (uses ``dd::display_config@1.0.0``):

The ``camera.size``, ``camera.left.stream``, and ``camera.right.stream`` fields are required:

.. code-block:: json

   {
     "type": "dd::display_config@1.0.0",
     "name": "dvrk_display",
     "sinks": ["glimage"],
     "camera": {
       "size": { "width": 640, "height": 480 },
       "left":  { "stream": "v4l2src device=/dev/video0" },
       "right": { "stream": "v4l2src device=/dev/video1" }
     }
   }

For **Mono Viewer** (uses ``dd::mono_config@1.0.0``):

The ``camera.stream`` field is required:

.. code-block:: json

   {
     "type": "dd::mono_config@1.0.0",
     "name": "dvrk_display_mono",
     "sinks": ["glimage"],
     "camera": {
       "stream": "v4l2src device=/dev/video0"
     }
   }

Most stereo-specific fields (crop dimensions, alignment shifts, color calibration, display offset, etc.) are best adjusted interactively using the :doc:`calibration_tool` rather than edited by hand. The calibration tool will populate and update these fields in the JSON file automatically.

Full Configuration Reference
----------------------------

**Stereo Configuration Example:**

.. code-block:: json

   {
     "type": "dd::display_config@1.0.0",
     "name": "dvrk_display",
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

**Mono Configuration Example:**

.. code-block:: json

   {
     "type": "dd::mono_config@1.0.0",
     "name": "dvrk_display_mono",
     "dvrk_console_namespace": "console",
     "overlay_alpha": 0.7,
     "sinks": ["glimage"],
     "unixfdsinks": [
       { "stream": "raw", "name": "raw_output" },
       { "stream": "overlay", "name": "overlay_output" }
     ],
     "camera": {
       "stream": "v4l2src device=/dev/video0",
       "size": { "width": 640, "height": 480 },
       "color": { "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0 }
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
     - **Required.** Must be ``"dd::display_config@1.0.0"`` (stereo) or ``"dd::mono_config@1.0.0"`` (mono). Validated on startup.
   * - ``name``
     - string
     - Name of the viewer instance used as the ROS node name. Default: ``"dvrk_display"``.
   * - ``dvrk_console_namespace``
     - string
     - ROS namespace of the dVRK console, used to discover teleop and PSM topics for
       the HUD overlay. Default: ``"console"``.
   * - ``overlay_alpha``
     - number
     - Opacity of the HUD overlay icons (0.0 = transparent, 1.0 = opaque). Default: ``0.7``.
   * - ``preserve_size``
     - boolean
     - **Stereo only.** When ``true``, upscales the cropped image back to the original frame dimensions
       without black borders. Default: ``true``.
   * - ``display_horizontal_offset_px``
     - integer
     - **Stereo only.** Horizontal pixel offset applied symmetrically to each eye's crop window so that
       the stereo display presents content at the surgeon's working depth. Set by the
       calibration tool (``[`` / ``]`` keys). Default: ``0``.
   * - ``sinks``
     - array
     - Display output sinks. ``"glimage"`` opens a single window (side-by-side stereo for stereo; standard window for mono);
       ``"glimages"`` opens two separate per-eye windows (stereo only). Default: ``[]``.
   * - ``unixfdsinks``
     - array
     - Zero-copy shared-memory outputs via Unix file-descriptor sockets. Each entry is
       an object with a mandatory ``"stream"`` key, an optional ``"name"`` label, and an optional
       ``"socket_path"`` (auto-generated from ``name`` and username when omitted).
       
       * **Stereo streams:** ``"left"``, ``"right"``, ``"stereo"``, or ``"overlay"``.
       * **Mono streams:** ``"raw"`` or ``"overlay"``.
   * - ``unixfd_socket_path``
     - string
     - *Deprecated shorthand.* Sets a single stereo/mono unixfd socket path. Prefer
       ``unixfdsinks`` for new configurations.
   * - ``extra_streams``
     - object
     - **Stereo only.** Optional picture-in-picture streams composited into the bottom of each eye.
       See `Extra Streams`_ below.


``camera`` Object
~~~~~~~~~~~~~~~~~

The required ``camera`` object groups all camera-related settings. Its fields differ depending on whether you are configuring a Stereo or Mono viewer.

Stereo Camera Settings (Required for ``stereo``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Mono Camera Settings (Required for ``mono``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Field
     - Type
     - Description
   * - ``camera.stream``
     - string
     - **Required.** GStreamer pipeline fragment for the mono camera.
   * - ``camera.size.width`` / ``camera.size.height``
     - integer
     - (Optional) Frame dimensions in pixels.
   * - ``camera.color``
     - object
     - Color correction applied via a ``videobalance`` GStreamer element.  Fields:
       ``brightness`` (−1.0–1.0, default 0.0), ``contrast`` (0.0–2.0, default 1.0),
       ``saturation`` (0.0–2.0, default 1.0), ``hue`` (−1.0–1.0, default 0.0).

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

Geometric Scaling & Size Preservation (Stereo Only)
---------------------------------------------------

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

Color correction is applied via ``videobalance`` GStreamer elements. For the stereo viewer, this uses ``camera.left.color`` and ``camera.right.color`` per eye. For the mono viewer, this uses ``camera.color``.

Adjusting ``brightness``, ``contrast``, ``saturation``, and ``hue`` corrects manufacturing differences or lighting imbalances. For the stereo viewer, use the :doc:`calibration_tool` ``c`` key to cycle through automatic matching modes.

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
- ``test_mono.json`` — Simple test configuration for the mono viewer.
- ``stereo_config.schema.json`` — Formal JSON Schema for offline stereo validation.
- ``mono_config.schema.json`` — Formal JSON Schema for offline mono validation.
