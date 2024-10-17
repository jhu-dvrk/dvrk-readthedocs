
USB foot pedals
===============

**Note:** not all USB foot pedals are supported at that point.  To
 easily integrate the foot pedal with cisst/SAW, we're using the
 [sawJoystick](https://github.com/jhu-saw/sawJoystick) component.  See
 README.md for the *sawJoystick* for more information.

Compilation
-----------

To compile the *sawJoystick* component, we strongly recommend to use
`wstool` to get all the source code under your catkin workspace.  You
can find the `.rosinstall` file for the *sawJoystick* code under the
`sawJoystick/ros` directory on
[github](https://github.com/jhu-saw/sawJoystick).
 
Configuration
-------------

First you will need to configure the dVRK console to use the *sawJoystick* component.  Your `console.json` file should contain:

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
