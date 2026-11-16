.. _config-pedals-usb:

.. include:: /includes/logic-view-console.rst

USB foot pedals
***************

For Linux USB pedals, use ``sawGenericInput`` with class ``mtsEvdev``.
Complete the :ref:`device setup and permissions <pedals-usb>` first.
If the component is missing from your workspace, follow
:ref:`Adding components to an existing workspace <vcs-add-components>`.

Input device configuration
==========================

Save a file named ``sawGenericInput-pedals.json`` in the dVRK configuration
search path, or use its absolute path in ``configure-parameter`` below.
This example selects the PCsensor FootSwitch Keyboard and names the three
provided button interfaces:

.. code-block:: json

   {
       "device": {
           "name": "PCsensor FootSwitch Keyboard",
           "vendor_id": "3553",
           "product_id": "b001"
       },
       "exclusive": true,
       "buttons": [
           {"key": "KEY_A", "interface": "Clutch", "quick_tap_ms": 300},
           {"key": "KEY_B", "interface": "Camera", "quick_tap_ms": 300},
           {"key": "KEY_C", "interface": "OperatorPresent", "quick_tap_ms": 0}
       ]
   }

Use the actual keys or joystick ``BTN_*`` codes reported by your pedals.
The ``sawGenericInputConfigQt`` configurator can observe those codes and save
a configuration. A stable ``/dev/input/by-id/...`` path can also be used as
the ``device`` value instead of the name/USB-ID selector.

A press sends a cisst ``prmEventButton::PRESSED`` event; release sends
``RELEASED``. Kernel key repeats are ignored. The clutch and camera mappings
also send ``CLICKED`` after release for a press lasting at most 300 ms;
clutch quick taps cycle selected PSM teleoperation. Set ``quick_tap_ms`` to
zero to disable click detection, as shown for operator presence.

System configuration
====================

Merge the following fields into your existing dVRK system configuration,
retaining its arms, teleoperation settings, and other components. The named
``console`` uses the three button interfaces from component ``Pedals``:

.. code-block:: json

   {
       "component_manager": {
           "components": [
               {
                   "shared-library": "sawGenericInput",
                   "class-name": "mtsEvdev",
                   "constructor-arg": {
                       "Name": "Pedals",
                       "Period": 0.002
                   },
                   "configure-parameter": "sawGenericInput-pedals.json"
               }
           ]
       },
       "consoles": [
           {
               "name": "console",
               "input_type": "COMPONENTS",
               "clutch": {"component": "Pedals", "interface": "Clutch"},
               "camera": {"component": "Pedals", "interface": "Camera"},
               "operator_present": {"component": "Pedals", "interface": "OperatorPresent"}
           }
       ]
   }

Each provided interface emits a standard cisst ``Button`` event. The component
can therefore be connected directly to the console without a ROS button
bridge. Keep the operator-present pedal pressed while teleoperating.

``exclusive: true`` captures all keys from the selected input node. On device
disconnect, the component releases all inputs and retries once per second.
After reconnecting, release any held pedal before pressing it again.
