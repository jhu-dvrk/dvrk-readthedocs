.. _dvrk-control-panel:

Control panel
#############

``control_panel`` is the user-facing interface for routine dVRK operation.  It
condenses the controls and status an operator normally needs at the surgeon
console.

The main ``dvrk_system`` application has a different purpose: it is the
engineering view used for system configuration, detailed component state,
diagnostics, and debugging.  ``control_panel`` complements it; it does not
replace the engineering interface.

.. figure:: /images/gui/control_panel_dark.png
   :width: 400px
   :align: center

   Control panel GUI (dark mode)


Starting the panel
******************

The panel can discover active ``@dvrk`` sockets without a configuration:

.. code-block:: bash

   ros2 run dvrk_console control_panel

Options are:

.. code-block:: text

   -c, --config <file>    control-panel JSON configuration
   -C, --console <name>   override the dVRK console namespace
   -s, --source <socket>  add a preferred video socket; may be repeated
   -a, --annotations     enable video annotations

For example:

.. code-block:: bash

   ros2 run dvrk_console control_panel \
     -C console \
     -s @dvrk:stereo_display:overlay

Operator functions
******************

The panel provides:

* system home and power-off controls;
* operating state, homed/busy state, and instrument name for active arms;
* a per-arm reset sequence that disables, enables, and homes the arm;
* global teleoperation enable;
* selection and unselection of discovered PSM and ECM teleoperations;
* current teleoperation state;
* teleoperation scale;
* system audio volume;
* embedded per-eye video selected from supported dVRK GStreamer sockets;
* optional drawing annotations for the final stereo overlay;
* persisted window monitor, fullscreen, video source, and light/dark theme.

The system power indicator is derived from active arm operating states.  The
power button publishes ``/system/home`` while the system is off and
``/system/power_off`` while it is on.  An arm reset is an asynchronous recovery
sequence with timeouts, not an emergency stop.

Configuration
*************

The current control-panel format is field based and does not require a
versioned ``type`` property:

.. code-block:: json

   {
     "name": "surgeon_console",
     "console": "console",
     "video_source": [
       "@dvrk:stereo_display:overlay"
     ]
   }

``name`` is also used as the key for persisted display settings.  ``console``
selects the ROS namespace.  Each ``video_source`` entry is a canonical dVRK
abstract socket name.

Configured sources are added first.  By default, the panel discovers the
side-by-side ``stereo_source:stereo``, ``stereo_alignment:stereo``, and
``stereo_display:overlay`` sockets and offers left and right eye views from
each. Whole side-by-side frames and unrelated sockets are not listed by
default. The most recently selected source is restored when available.

The installed ``control_panel.schema.json`` describes this format.

Persisted user settings
***********************

The panel stores display and user preferences in an INI-style file under the
user configuration directory:

.. code-block:: text

    ~/.config/dvrk_display/<name>_control_panel_gui.ini

``<name>`` comes from the control-panel configuration field ``name`` (default
``dvrk_display``).  Non-alphanumeric characters are normalized to ``_`` for the
filename.

Current persisted keys include:

* ``[window] monitor``: selected monitor index.
* ``[window] fullscreen``: fullscreen state.
* ``[appearance] dark_mode``: light/dark mode selection.
* ``[video] source``: last selected fully qualified video socket.
* ``[touchscreen] monitor_N``: per-monitor touchscreen flag (``N`` is monitor
   index).
* ``[touchscreen] device_name``: optional persisted XInput device name; an
   empty value selects automatic detection.

The panel saves settings when options are changed and again during normal
application exit.

Video preview
*************

The preview uses ``unixfdsrc`` directly and therefore remains local to the
computer hosting the video producer. It does not subscribe to a ROS image
topic. The robot controls remain usable when no video source is available.

For a graph of the negotiated preview pipeline, see
:ref:`dvrk-gstreamer-dot-files`.

Touchscreen support
*******************

.. figure:: /images/dvrk_console/dvrk_console_remote_control_touch.jpeg
   :width: 400px
   :align: center

   Control panel with touchscreen at JHU

The wrench menu provides a per-monitor touchscreen toggle and input-device
selection:

* ``Touchscreen -> Is touchscreen``
* ``Touchscreen -> Not a touchscreen``
* ``Touchscreen -> Input Device -> Automatic`` or a specific XInput pointer
  device

This flag is stored per monitor index in the user settings file.  When enabled,
the panel maps the selected touchscreen pointer device to the current monitor
output using the XInput coordinate transformation matrix.

Runtime behavior:

* Mapping is applied for monitors marked as touchscreen.
* Mapping is re-applied when moving the window to another monitor.
* Monitor changes are polled periodically while the panel is running.
* Mapping failures are non-fatal and reported as warnings on stderr.

Implementation notes for deployment:

* Automatic mapping enumerates XInput2 slave pointer devices directly. Devices
  exposing touch or absolute-valuator classes are preferred, followed by names
  containing touch, digitizer, stylus, tablet, or known controller identifiers
  such as WDT.
* The selected device name is persisted in the user settings file. XInput
  numeric IDs are discovered at runtime because they can change between boots.

Video annotations
*****************

Annotations are off by default. Run ``ros2 run dvrk_console control_panel -a``
to enable them with the default sockets, or include an ``"annotations": {}``
object in the control-panel configuration. Select **Annotate** below the video
preview to draw with a finger, stylus, or mouse. Annotations work on any
monitor. The **Touchscreen** menu configures touch input for a monitor; it does
not enable or disable annotations.

The **Video Input** button below Scale and Volume offers left and right eye
views from the available side-by-side ``stereo_source:stereo``,
``stereo_alignment:stereo``, and ``stereo_display:overlay`` sockets. It does
not offer whole side-by-side frames or unrelated sockets by default.
**Annotate** switches to the final ``@dvrk:stereo_display:overlay`` stream,
detects per-eye dimensions, and previews the selected eye. Drawing coordinates
therefore match the final overlay pixels.

Draw with a finger, stylus, or the left mouse button. Select the eye, color,
and pen width in the row below the preview. **Both** applies new strokes,
Undo, and Clear to both eyes at the same image coordinates. Otherwise these
operations apply only to the selected eye. **All** clears both eyes; **Show**
hides or restores all marks without deleting them. Drawing is suspended while
marks are hidden. Identical marks in both eyes are screen-plane annotations,
not anatomical stereo correspondences. Marks remain fixed in image
coordinates; they do not track camera or tissue motion.

The panel publishes one transparent BGRA side-by-side LR stream at 30 Hz on
``@dvrk:control_panel:annotations``. Output starts when overlay video is
received and continues when another video is selected through **Video Input**.
A disconnected or stalled preview clears marks after one second; resolution
changes also clear marks. **Retry** restarts preview and output initialization
after a socket error.

To mix drawings into the display, enable the stereo AR input in
:ref:`dvrk-stereo-display-configuration`. AR inputs run independently of the
camera pipeline. Before the panel starts, or after it closes, the display
supplies transparent frames while reconnecting. A stalled AR source becomes
transparent after one second. Color keying is not needed for this output.

A configuration file can enable annotations with ``"annotations": {}`` or
override the output socket with ``annotations.gst_output``, using a canonical
``@dvrk:role:name`` reference. Set ``ar.stereo.gst_input`` in the display
configuration to the same name. The annotation preview remains on the final
overlay socket: selecting a **Video Input** source does not change it. See
``share/config/control_panel_annotations.json`` for an example.

Testing
~~~~~~~

Configure with ``BUILD_TESTING=ON``, build ``annotation_model_test``,
``optional_ar_input_test``, and ``annotation_panel_test``, then run ``ctest`` in
the package build directory. The panel test uses an offscreen GTK window and
is skipped when no GTK display backend is available. It checks pointer drawing
and clearing through real output sockets; physical touch and stylus input
still need verification on the target panel.
