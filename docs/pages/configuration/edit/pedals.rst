
Foot pedals
###########

.. _config-pedals-original:

Original and compatible
***********************

The default digital inputs for the daVinci :ref:`Classic foot pedals
<pedals-original>` (and the :ref:`compatible ones
<pedals-compatible>`) are defined in share IO configuration files.
These depends on which controller is used, both :ref:`board Id
<board-id>` and :ref:`hardware version <controller-classic-exterior>`
so we provide multiple configuration files.

All the default foot pedal IO configuration files are in the
*sawIntuitiveResearchKit* repository, under ``io/share``.

For example, if your pedals are connected to a MTML controller with a
FPGA version 1 or 2, your console JSON file should have:

.. code-block:: JSON

    "io": {
        "footpedals": "io/sawRobotIO1394-MTML-foot-pedals.xml"
    }

If the foot pedals are connected to a MTMR controller with a FPGA version 3 (i.e. with DQLA), your console JSON file should have:

.. code-block:: JSON

    "io":
    {
        "footpedals": "io/sawRobotIO1394-MTMR-foot-pedals-DQLA.xml"
    }

See also :ref:`dMIB IOs <dmib-io>`.

.. _config-pedals-usb:

USB "joystick"
**************

Ignorxe this section if you are not using a :ref:`USB "joystick" foot
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
