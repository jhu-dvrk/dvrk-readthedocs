.. _pedals-usb:

USB foot pedals
===============

.. warning::

   Not all USB foot pedals are supported at that point.  To easily
   integrate the foot pedal with cisst/SAW, we're using the
   `sawJoystick <https://github.com/jhu-saw/sawJoystick>`_ component.
   See README.md for the *sawJoystick* for more information.  You need
   to make sure the pedals you're about to buy are "joystick"
   compatible.  Some pedals only support "hid".

If your foot pedals are HID compatible only, please reach out to the
dVRK developers, we might be able to add support.

Compilation
-----------

To compile the *sawJoystick* component, we strongly recommend to use
``vcs`` to get all the source code under your ROS workspace.  You can
find the ``.vcs`` file for the *sawJoystick* code under the
https://github.com/jhu-saw/vcs.

Configuration
-------------

To configure your console, see :ref:`foot pedals configuration
<config-pedals-usb>`
