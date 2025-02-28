.. _config-pedals-usb:

.. include:: /includes/logic-view-console.rst

USB "joystick" foot pedals
**************************

Ignore this section if you are not using a :ref:`USB "joystick" foot
pedal <pedals-usb>`.

After you made sure *sawJoystick* is compiled in your ROS workspace,
you will need to configure the dVRK console to use the *sawJoystick*
component.  Your ``console.json`` file should contain:

.. code-block:: JSON

    "component-manager": {
        "components":
        [
            {
                "shared-library": "sawJoystick",
                "class-name": "mtsJoystick",
                "constructor-arg": {
                    "Name": "joystick"
                },
                "configure-parameter": "misc/sawJoystickConfiguration.json"
            }
        ]
    }
    ,
    "console-inputs":
    {
        "operator-present": {
            "component": "joystick",
            "interface": "OperatorPresent"
        }
        ,   "clutch": {
            "component": "joystick",
            "interface": "Clutch"
        }
    }


Then you need to configure the *sawJoystick* component so that buttons
are mapped to names that match the dVRK foot pedal names
(e.g. "OperatorPresent", "Clutch"...).  We provide an example of
*sawJoystick* configure for the dVRK in
`share/misc/sawJoystickConfiguration.json`:


.. code-block:: JSON

    "converters":
    [
        {
            "type": "interface-provided-button",
	    "index-input": 1,
            "interface-name": "OperatorPresent"
        }
	,
        {
            "type": "interface-provided-button",
	    "index-input": 0,
            "interface-name": "Clutch"
        }
    ]
    ,
    "device": "/dev/input/js0"

To test which "device" and "index-input" to use, you can run the
example application that comes with *sawJoystick*:
`sawJoystickQtExample`.  To test different devices, you can use the
`-d` option (e.g. `-d /dev/input/js0`).
