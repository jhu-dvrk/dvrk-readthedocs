.. _config-head:

Head sensor
***********

To enable any teleoperation, the dVRK needs to know if the operator is
present and holding the MTMs.  On the clinical systems, there is a
sensor at the level of the stereo display to detect if the operator's
head is facing the display. On the first versions of dVRKs, we didn't
have a head sensor so we used one of the foot pedals as a deadman
switch. The operator was considered present if and only if the "COAG"
pedal was pressed.  Having to press the pedal continuously is quite
inconvenient so we came up with a few alternatives.

.. note::

   Your systems should have only one head sensor, make sure you refer
   to the correct section.

See also :ref:`dMIB IOs <dmib-io>`.


.. _config-head-original:

da Vinci head sensor
====================

If you've built :ref:`your custom cable <head-original>` to interface
with the original head sensor, you can modify your console JSON
configuration file.

Assuming that you're connecting your head sensor to the MTMR
controller, always on the ``DOF 1`` connector, you just need to add
the following line in your console JSON configuration file:

.. code-block:: json

   "operator-present": {
     "io": "io/sawRobotIO1394-MTMR-dv-head-sensor.xml"
   }


.. warning::

   The board numbering on the software side starts with index 0
   (e.g. in XML IO configuration file) but the labels on the back of
   the controller start at index 1.  Keep this in mind if you plan to
   connect the head sensor to another "DOF" and create your own XML
   configuration files.


.. _config-head-goovis:

Goovis head sensor
==================

For the :ref:`Goovis HMD <goovis>`, we added a lightweight HID class
to the dVRK code to get access to the head sensor.  The code can be
found in ``component/code/mtsHIDHeadSensor.cpp``.  The component has a
default configuration file under ``share/hid/goovis-hd.json`` which
contains:

.. code-block:: json

   {
     "id_vendor": "880a",
     "id_product": "3501",
     "index_data": 17
   }

Then in your console's JSON configuration file you should add:

.. code-block:: json

   "operator-present":
   {
     "hid": "hid/goovis-hd.json"
   }


.. _config-head-dvrk:

dVRK head sensor
================

Assuming you've built :ref:`your custom head sensor <head-dvrk>`, you can
modify your console JSON configuration file.

There is no specific IO configuration to perform at that point
provided that you connect the head sensor on the same controller as
the foot pedals (for **rev 1.5** and above, HEAD is already included
in the IO foot pedal XML files)


Update your JSON config file to set the presence sensor or point to
the IO foot pedal configuration file (rev 1.5+)

  .. code-block:: json

     "console-inputs": {
       "operator-present": {
          "component": "io",  // hard coded in source code, file mtsIntuitiveResearchKitConsole.cpp
          "interface": "Head" // name of the button you want to use, defined in sawRobotIO1394 configuration file
       }
     }
