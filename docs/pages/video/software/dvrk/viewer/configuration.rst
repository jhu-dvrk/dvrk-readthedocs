Configuration Schema
====================

The ``dvrk_display`` heavily relies on a well-defined JSON schema (``sv::dvrk_display_config@1.0.0``) to control the dimensions, color-balancing arrays, and geometry.

Getting Started
---------------

To create a new configuration, start with a minimal JSON file containing only the essential fields:

.. code-block:: json

   {
     "type": "sv::dvrk_display_config@1.0.0",
     "name": "dvrk_display",
     "left_stream": "v4l2src device=/dev/video0 ! video/x-raw,width=1920,height=1080",
     "right_stream": "v4l2src device=/dev/video1 ! video/x-raw,width=1920,height=1080"
   }

Most other fields (crop dimensions, shifts, color calibration, etc.) are best adjusted interactively using the :doc:`configurator_tool` rather than edited by hand. The configurator will populate and update the remaining fields in the JSON file automatically.

Full Configuration Reference
----------------------------

After tuning with the configurator, a complete configuration file will look like:

.. code-block:: json

   {
     "type": "sv::dvrk_display_config@1.0.0",
     "name": "dvrk_display",
     "dvrk_console_namespace": "console",
     "ros_image_publishers": ["stereo"],
     "overlay_alpha": 0.7,
     "original_width": 1920,
     "original_height": 1080,
     "crop_width": 1794,
     "crop_height": 1009,
     "horizontal_shift_px": -72,
     "vertical_shift_px": -7,
     "preserve_size": true,
     "left_stream": "...",
     "right_stream": "...",
     "sinks": ["glimage"],
     "unixfd_socket_path": "/tmp/stereo_viewer.sock",
     "left_color": { "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0 },
     "right_color": { "brightness": 0.0, "contrast": 1.0, "saturation": 1.0, "hue": 0.0 }
   }

Field Reference
~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 25 10 65

   * - Field
     - Type
     - Description
   * - ``type``
     - string
     - **Required.** Must be ``"sv::dvrk_display_config@1.0.0"``. Validated on startup.
   * - ``name``
     - string
     - Name of the viewer instance. Used as the ROS node name. Default: ``"dvrk_display"``.
   * - ``left_stream``
     - string
     - **Required.** GStreamer pipeline fragment for the left camera source. Must end with a raw video pad.
   * - ``right_stream``
     - string
     - **Required.** GStreamer pipeline fragment for the right camera source. Must match left dimensions and frame rate.
   * - ``dvrk_console_namespace``
     - string
     - ROS namespace of the dVRK console. Used to discover teleop and PSM topics for the overlay. Default: ``"console"``.
   * - ``ros_image_publishers``
     - array
     - List of ROS image streams to publish. Accepted values: ``"left"``, ``"right"``, ``"stereo"``. Default: ``[]`` (none).
   * - ``overlay_alpha``
     - number
     - Opacity of the HUD overlay icons (0.0 = transparent, 1.0 = opaque). Default: ``0.7``.
   * - ``original_width``
     - integer
     - Native width of each camera frame in pixels before cropping.
   * - ``original_height``
     - integer
     - Native height of each camera frame in pixels before cropping.
   * - ``crop_width``
     - integer
     - Width of the output region after alignment shift and crop. Defaults to ``original_width``.
   * - ``crop_height``
     - integer
     - Height of the output region after alignment shift and crop. Defaults to ``original_height``.
   * - ``horizontal_shift_px``
     - integer
     - Horizontal pixel offset applied to the right eye relative to the left. Default: ``0``.
   * - ``vertical_shift_px``
     - integer
     - Vertical pixel offset applied to the right eye relative to the left. Default: ``0``.
   * - ``preserve_size``
     - boolean
     - When ``true``, upscales the cropped image to fill the original frame dimensions without black borders. Default: ``true``.
   * - ``sinks``
     - array
     - Output sink selection. ``"glimage"`` produces a single side-by-side stereo window; ``"glimages"`` produces two separate windows (one per eye). Default: ``["glimage"]``.
   * - ``unixfd_socket_path``
     - string
     - Path to a Unix file-descriptor socket for zero-copy shared-memory transport (``unixfdsink``/``unixfdsrc``). When set, the combined stereo frame is also published on this socket. Leave empty or omit to disable.
   * - ``left_color``
     - object
     - Color correction for the left eye: ``brightness``, ``contrast``, ``saturation``, ``hue``. See below.
   * - ``right_color``
     - object
     - Color correction for the right eye. Same sub-fields as ``left_color``.

Geometric Scaling & Size Preservation
-------------------------------------

GStreamer natively crops images relative to the stream parameters. When defining the ``crop_width`` and ``crop_height`` parameters, the underlying engine offsets these calculations using ``horizontal_shift_px`` and ``vertical_shift_px``.

When ``preserve_size`` is ``true``, the viewer computes an inner-crop that matches the native source aspect ratio and upscales the result to fill the original frame dimensions, eliminating black borders without distorting the aspect ratio.

Color Calibration Maps
----------------------

Real-world endoscopes often have slight manufacturing or lighting deviations. The ``left_color`` and ``right_color`` sub-dictionaries inject native ``videobalance`` GStreamer elements onto the pipeline, altering ``brightness``, ``contrast``, ``saturation``, and ``hue`` in real time. These values are best set interactively using the :doc:`configurator_tool` color calibration mode rather than edited by hand.

Example Configurations
----------------------

The package ships example configuration files under ``share/``:

- ``dvrk_display.json`` — basic configuration template for standard V4L2 stereo sources.
- ``decklink_gooviz.json`` — pre-configured for Blackmagic Decklink capture cards with Gooviz displays.
- ``dvrk_display_config.schema.json`` — the formal JSON Schema definition for validation.

