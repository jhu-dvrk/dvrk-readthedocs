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

The panel can discover active ``@dvrk_gst`` sockets without a configuration:

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
     -s stereo_display:overlay

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
       "stereo_display:overlay"
     ]
   }

``name`` is also used as the key for persisted display settings.  ``console``
selects the ROS namespace.  Each ``video_source`` entry accepts a short name,
``role:name``, or a fully qualified abstract socket.

Configured sources are added first.  At startup the panel scans for every
other active ``@dvrk_gst`` socket and adds those choices without duplication.
The most recently selected source is restored when it is available.

The installed ``control_panel.schema.json`` describes this format.

Video preview
*************

The optional preview uses ``unixfdsrc`` directly and therefore remains local
to the computer hosting the video producer.  It does not subscribe to a ROS
image topic.  If no source is configured or discovered, the video pane is
hidden and the robot controls remain usable.

Relationship to the stereo display
***********************************

For an operator preview with HUD annotations, configure ``stereo_display`` to
publish its ``overlay`` output:

.. code-block:: json

   {
     "unixfdsinks": [
       {"socket": "overlay"}
     ]
   }

Then select ``stereo_display:overlay`` in the panel.  Use the unannotated
``stereo_display:stereo`` output when the HUD is not wanted.
