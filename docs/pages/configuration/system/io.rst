.. _config-io:

.. include:: /includes/logic-view-console.rst

IOs
***

`Documention based on schema
<../../../schemas/dvrk-IO.html>`_ is also
available for reference.

.. raw:: html

    <iframe style="width: 100%; height: 80vh" frameBorder="0" src="../../../schemas/dvrk-IO.html"></iframe>
    
Name
====

The name of the IO component is required. It is needed by the arms,
inputs and any other component that need to know which IO port should
be used. There is no default name but we recommend using something
short like "IO", "IO_1"...

Port
====

The port defines the type of :ref:`connection between the PC and the
dVRK controllers <connectivity>`. For anyone using FireWire for all
the communications, the setting should be ``fw``.  If your system uses
FireWire between the dVRK controller and Ethernet to bridge to the PC,
use ``udpfw``. Finally, if your system is recent (post 2024) and only
uses :ref:`FPGA V3 <fpga>`, you can connect everything using Ethernet
cables and use ``udp``.

If you have multiple FireWire or Ethernet adapters on your PC, you
will need to add the FireWire port number or Ethernet IP address after
the port's type (e.g. ``fw:1``).

Having multiple IOs is very convenient to handle systems with multiple
surgeon's consoles. For example, one might want a system with 2 MTMLs
and 2 MTMRs. This issue is that each arm assumes a default :ref:`board
Id <board-id>` based on the arm's type. Therefore, both MTMLs would
use the same board Id(s) and create a conflict if used on the same IO
port.

.. caution::

   Do no declare multiple IOs using the same port in a single system
   configuration file!

FireWire protocol
=================

You can also specify the FireWire protocol used to communicate between
the controllers:

.. code-block:: JSON

    {
      "protocol": "sequential-read-broadcast-write" // default
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

One can change the default refresh rate (in this example, 1/2 millisecond).
It is recommended to not override this option and use the default
value.

In most cases, the PID components run in the same thread as the IO, so
changing the IO period also changes the PID period.  See
:ref:`software architecture <threads>`.

.. code-block:: json

   {
     "period": 0.0005 // in seconds
   }

Watchdog time-out
=================

You can override the watchdog time-out using:

.. code-block:: json

   {
     "watchdog_timeout": 0.03 // in seconds
   }

The watchdog time-out is set on the FPGA-QLA controllers.  If the
controllers don't receive any message for a period exceeding the
watchdog time-out, they will automatically turn off the power on all
motors.  This is used if the physical communication is lost (unplugged
wire) or if the application has crashed or is not sending commands
fast enough.  The maximum value for the watchdog time-out is 300 ms.
Setting the time-out to zero turns off the watchdog and is not
recommended.  This field is optional, and it is recommended to not
override the default.


Extra configuration files
=========================

The IO component can also be configured to support custom hardware
using the controllers spare digital and analog IOs (see :ref:`dMIB IOs
<dmib-io>`). This setting shouldn't be used for the hardware supported
by the dVRK software stack such as foot pedals, head sensor, focus
controller...
