.. _setup-classic:

*******
Classic
*******

dMIB modification for setup joints switch ECM pre 2015
======================================================

The setup joint switch/button on the ECM is not using the same digital
input as the setup joint switch on the PSMs.  This was unfortunately
discovered after the dMIB were designed (pre 2015 revisions).  In
other words, you might have to modify the dMIB to short a couple pins.
You will need someone in house who can do some soldering.

Using the sawRobotIO1394 Console you should be able to monitor the
switch events, i.e. press and release the different buttons on the ECM
arm for a little bit and monitor the changes in the "Buttons"
widget/window.

Manipulator switch ![ECM Manipulator switch](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ecm-arm-switch.jpg)

SUJ switch ![ECM Arm switch](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ecm-suj-switch.jpg)

IO Widget ![sawRobotIO1394QtConsole IO](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/gui/dvrk-gui-ecm-io.png)

The Arm (aka manipulator) switch should work and the SUJ shouldn't
until you hack the dMIB.  To modify the dMIB, follow [these
instructions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci-dMIB-pre-2015).
