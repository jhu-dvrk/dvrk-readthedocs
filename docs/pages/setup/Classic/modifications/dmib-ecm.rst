.. _dmib-ecm-pre-2015:

dMIB modification for setup joints switch ECM pre 2015
******************************************************

.. note::

   The SUJ switch for the ECM is physically connected to the ECM
   Classic controller. If your ECM controller was manufactured before
   2015, SUJ switch will not work unless you modify the dMIB

The setup joint switch/button on the ECM is not using the same digital
input as the setup joint switch on the PSMs.  This was unfortunately
discovered after the dMIB were designed (pre-2015 revisions).  In
other words, you might have to modify the dMIB to short a couple pins.
You will need someone in house who can do some soldering.

Using the sawRobotIO1394 Console you should be able to monitor the
switch events, i.e. press and release the different buttons on the ECM
arm for a bit and monitor the changes in the "Buttons"
widget/window.

.. figure:: /images/Classic/ECM/ecm-arm-switch.jpg
   :width: 400
   :align: center

   ECM manipulator switch

.. figure:: /images/Classic/ECM/ecm-suj-switch.jpg
   :width: 400
   :align: center

   ECM SUJ switch

.. figure:: /images/gui/dvrk-gui-ecm-io.png
   :width: 600
   :align: center

   ECM IO widgets

