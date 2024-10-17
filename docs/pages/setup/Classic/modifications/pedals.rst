Foot pedals
***********

Introduction
============

This page presents alternative foot pedals that can be used the dVRK controllers and software.  There are multiple cases where one might need some extra foot pedals:
* Splitting your system into two systems, e.g. using the MTMR and PSM1 for one setup and the MTML and PSM2 for another setup.
* Using extra inputs from a patient's side assistant.
* Using the dVRK software without any dVRK controller.  For example, using a Force Dimension haptic device on the surgeon's side and the patient cart in simulation.

We present two different options that are supported with the dVRK controllers and/or software:
* Foot pedals that connect to the dVRK controllers
* USB foot pedals with joystick emulation (most cheap foot pedals)

Orginal da Vinci foot pedals
============================

Description
-----------

The original da Vinci foot pedal tray has 5 pedals.  From left to right: CLUTCH, CAMERA, Focus +/-, GOAG and BI-COAG.  The pedals CLUTCH and CAMERA are used to clutch the MTMs and swith to-from PSM vs ECM teleoperation.  The Focus +/- is used to control the endoscope focus (there is a custom cable for the dVRK) and cauterization (mono-polar and bi-polar).  For the dVRK, we tend to use the COAG pedal as a dead-man switch in lieu of the head sensor.

Removing the foot pedal tray
----------------------------

* Foot pedal cable can be found on the front of the master console, under the stereo display and the cover needs to be removed.  Unplug the bottom part and connect it to the dVRK controller.  This cable is rather short so one might consider investing in an extension cable (e.g. https://smile.amazon.com/Monoprice-6ft-DB15-Molded-Cable/dp/B002LWJ7TA).  Note: if you want to remove the foot pedals, see [Foot Pedals](/jhu-dvrk/sawIntuitiveResearchKit/wiki/FootPedals).

![Foot pedals connector](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/full/master-console-pedals-cable.jpg)

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/removing-footpedals-console.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/removing-footpedals-console.jpg" width="450"></a>

Removing the tray from the console is a bit tricky.  You will first need to unplug the cable from both the foot pedal and the console.  On the console side, the connector is hidden under the plastic cover under the stereo display.  This plastic cover is holding with clips so you just have to pull it away.

Then you should open the cable management bar to free the cable.  Remove the screw at the pivot point, flip the bar upside-down and remove the small philips screws.  At that point, you can remove the black round grommet so you can push the round connector inside the base.  The grommet is glued so you'll need to pry it out using a flat head screw driver.  Make sure you don't damage the cable.

The next step is to remove the metal  cover placed under the console.  There is a cavity under the base but it has a cover holding with 4 9/64 hex screws.  You will need to reach out under the console and find the four screws by touch.  There is just enough space between the floor and the bottom to use a small allen wrench.  Removing these 4 screws is the trickiest part of the process.

Once the cover plate is removed, push the round connector in the round opening, push the rectangular connector in the rectangular opening and you'll be able to pull the cable from under.

dVRK controller compatible foot pedals
======================================

The goal of this section is to show how to build some dVRK compatible foot pedals.  These can only be used with the dVRK controllers and can't be connected directly to a PC.  This can be useful if you're splitting your dVRK into two systems (e.g., one MTM/PSM on one controller and the other MTM/PSM an another controller) or if you use a spare PSM with an alternate master arm (e.g., PSM and Force Dimension master).

The design described below is based on up to 3 pairs of foot pedals.  One can use one, two or three pairs as needed.  The wiring allows to swap the pedals without any software reconfiguration.  This configuration is also pin compatible with the real da Vinci foot pedals.  By default, the first pair of foot pedals is wired as the right two pedals on the daVinci (from left to right: bi-coag and coag) and the second pair of pedals is wired as the left two pedals (from left to right: clutch and camera):

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/dVRK-foot-pedals.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/dVRK-foot-pedals.jpg" width="450"></a>

Material
--------

 * One (or two or three) pairs of foot pedals.  We found that these pedals are nicely built and heavy enough to not slide on the floor when used: https://smile.amazon.com/dp/B077NM69DL   The following model might work as well but we didn't try: https://smile.amazon.com/Plastic-Double-Action-Switch-Pedal/dp/B077NK8XYZ
 * Cables: https://smile.amazon.com/Monoprice-6ft-DB15-Molded-Cable/dp/B002LWJ7TA  You will need 2 to 4 cables.

Wiring
------

To minimize the amount of soldering we simply cut one DB 15 extension cable in two (one cable per foot pedal).  The wiring for two foot pedals (i.e., without the optional 3rd foot pedal for camera +/-) can be seen in the following picture (credit: Christian Hernandez):

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/dVRK-foot-pedal-wiring.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/dVRK-foot-pedal-wiring.jpg" width="450"></a>

The complete wiring can be found in this [PDF file](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/dVRK-foot-pedal-wiring.pdf) or in [Altium Designer format](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/foot/Footpedal-Wiring.SchDoc).

Software configuration
----------------------

There is no software specific configuration, just use the IO files
used with the Intuitive Surgical foot pedals.

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
