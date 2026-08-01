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

For example:

.. code-block:: bash

   ros2 run dvrk_console control_panel \
     -C console \
     -s @dvrk:stereo_display:stereo

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
* optional embedded video selected from active dVRK GStreamer sockets;
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
       "@dvrk:stereo_display:stereo"
     ]
   }

``name`` is also used as the key for persisted display settings.  ``console``
selects the ROS namespace.  Each ``video_source`` entry is a canonical dVRK
abstract socket name.

Configured sources are added first.  At startup the panel scans for every
other active ``@dvrk`` socket and adds those choices without duplication.
The most recently selected source is restored when it is available.

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

The panel saves settings when options are changed and again during normal
application exit.

Video preview
*************

The optional preview uses ``unixfdsrc`` directly and therefore remains local
to the computer hosting the video producer.  It does not subscribe to a ROS
image topic.  If no source is configured or discovered, the video pane is
hidden and the robot controls remain usable.

For a graph of the negotiated preview pipeline, see
:ref:`dvrk-gstreamer-dot-files`.

Touchscreen support
*******************

The wrench menu provides a per-monitor touchscreen toggle:

* ``Touchscreen -> Is touchscreen``
* ``Touchscreen -> Not a touchscreen``

This flag is stored per monitor index in the user settings file.  When enabled,
the panel attempts to map the first detected touchscreen pointer device to the
current monitor output using ``xinput map-to-output``.

Runtime behavior:

* Mapping is applied for monitors marked as touchscreen.
* Mapping is re-applied when moving the window to another monitor.
* Monitor changes are polled periodically while the panel is running.
* Mapping failures are non-fatal and reported as warnings on stderr.

Implementation notes for deployment:

* Automatic mapping relies on ``xrandr`` and ``xinput``.
* The current implementation matches monitor geometry against ``xrandr`` output
   and uses the first matching touchscreen-like pointer device (excluding
   touchpads).
