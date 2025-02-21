.. _config-io:

IOs
***

Port
====

Period
======

One can change the refresh rate (in this example, 1/2 millisecond).
It is recommended to not include this section and use the default
values.

In most cases, the PID components run in the same thread as the IO, so
changing the IO period also changes the PID period.  See [software
architecture](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Software-Architecture).

.. code-block:: json
		
   "io": {
     "period": 0.0003 // in seconds
   }

Watchdog time-out
=================

In **version 1.5** and higher, you can specify the watchdog time-out using:

.. code-block:: json
		
   "io": {
     "watchdog-timeout": 0.03 // in seconds
   }

The watchdog time-out is set on the FPGA-QLA controllers.  If the
controllers don't receive any message for a period exceeding the
watchdog time-out, they will automatically turn off the power on all
motors.  This is used if the physical communication is lost (unplugged
wire) or if the application has crashed or is not sending commands
fast enough.  The maximum value for the watchdog time-out is 300 ms.
Setting the time-out to zero turns off the watchdog and is not
recommended.  This field is optional and it is recommended to not
override the default.

Foot pedals
===========

Also in **version 1.5**, IOs configuration for the foot pedals can
(and should) be separated from the arm configuration. The option
``"io": { "footpedals": }`` allows user to re-use a predefined
configuration for the foot pedals IO configuration.  For example, if
the foot pedals are connected to an MTMR controller, you will need:

.. code-block:: json
		
   "io": {
      "footpedals": "io/sawRobotIO1394-MTMR-foot-pedals.xml"
   }

