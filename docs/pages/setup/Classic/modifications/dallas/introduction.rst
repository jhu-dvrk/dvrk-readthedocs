Introduction
============

The da Vinci instruments can be automatically identified when inserted
in the sterile adapter using the 'add-only' chip embedded in the tool
(aka Dallas chip, DS2505).  This feature was not supported in early
versions of the dVRK Classic both from a software and hardware
perspective.

.. note::

   Instrument detection is supported without any modification for all
   dVRK Si controllers and any dVRK Classic controller shipped
   after 2019.

To retrieve the instrument tool type, the dVRK Classic controllers can
use two different approaches:

* **Dallas driver interface (DS2480B):** The FPGA communicates
  serially with the DS2480B chip, which then communicates with the
  DS2505 chip in the tool using a 1-wire interface.  The DS2480B chip
  is either on the dMIB (Rev F or greater) or on a dongle that is
  attached to a connector on the rear of the controller (see
  requirements below).

* **FPGA 1-wire interface:** In this scenario, the FPGA/QLA
  communicates directly with the tool's DS2505 chip.  This method
  requires QLA Version 1.4 or greater.

You can test the instrument detection using the command line tool
``instrument`` provided with the low level software (along ``qladisp``
in ``AmpIO``).  The ``instrument`` program will dump the memory from
the instrument in a text file.  Make sure you have an instrument
properly seated in the sterile adapter before launching the program.

.. code-block:: bash

   instrument [-pP] <board num>

You can also bypass the automatic instrument detection in the
:ref:`PSM configuration file <config-dallas>`.
