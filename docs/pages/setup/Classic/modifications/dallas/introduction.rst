Introduction
============

The da Vinci instruments can be automatically identified when inserted
in the sterile adapter using the `add-only` chip embedded in the tool
(aka Dallas chip, DS2505).  This feature was not supported in early
versions of the dVRK both from a software and hardware perspective.

To retrieve the instrument tool type, the dVRK hardware can use two
different approaches:

* **Dallas driver interface (DS2480B):** The FPGA communicates
  serially with the DS2480B chip, which then communicates with the
  DS2505 chip in the tool using a 1-wire interface.  The DS2480B chip
  is either on the dMIB (Rev F or greater) or on a dongle that is
  attached to a connector on the rear of the controller (See
  requirements below).

* **FPGA 1-wire interface:** In this scenario, the FPGA/QLA
  communicates directly with the tool's DS2505 chip.  This method
  requires QLA Version 1.4 or greater.

.. note::

   If your dVRK controller was shipped after 2019, you do not need to
   modify anything.
