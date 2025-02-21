.. _config-io:

IOs
***

Port
====


.. _config-pedals-original:

Foot pedals
===========

The default digital inputs for the da Vinci :ref:`Classic foot pedals
<pedals-original>` (and the :ref:`compatible ones
<pedals-compatible>`) are defined in shared IO configuration files.
These depend on which controller is used, both :ref:`board Id
<board-id>` and :ref:`hardware version <controller-classic-exterior>`
so we provide multiple configuration files.

All the default foot pedal IO configuration files are in the
|sawIntuitiveResearchKit| repository, under ``io/share``.

For example, if your pedals are connected to a MTML controller with an
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


FireWire protocol
=================

In **version 1.4** and higher, you can also specify the FireWire
protocol used to communicate between the PC and the controllers:

.. code-block:: JSON
		
   "io": {
      "firewire-protocol": "sequential-read-broadcast-write" // default
    }

The following protocols are supported:

* ``sequential-read-write``: the PC reads data from each board (2
  FPGA/QLA per controller), performs its computations (conversions,
  safety checks, PID, ...) and then writes sequentially to each board
  (N reads, N writes). This is the only protocol supported on older
  firmware (3 and below).

* ``sequential-read-broadcast-write``: the PC reads sequentially but
  performs a single write for all the boards. The write message is an
  array of values sent to the boards, each board access the data it
  needs by index (N reads, 1 write). This is the default protocol for
  the dVRK controllers with firmware 4 and above.

* ``broadcast-query-read-write``: the PC sends a single
  query/synchronization to all boards, read values are aggregated in
  single packet over the bus and there's a single write (1 query, 1
  read, 1 write). This is the fastest protocol available but some
  FireWire cards seem to have trouble synchronizing the read packets.
  You will have to test it on your hardware to see if it supports this
  protocol or not.

  
Period
======

One can change the refresh rate (in this example, 1/2 millisecond).
It is recommended to not override this option and use the default
value.

In most cases, the PID components run in the same thread as the IO, so
changing the IO period also changes the PID period.  See
:ref:`software architecture <threads>`.

.. code-block:: json

   "io": {
     "period": 0.0005 // in seconds
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

