.. _fpga:

.. include:: /includes/logic-view-fpga.rst

****************
Logic board FPGA
****************

Introduction
############

.. _fpga-versions:

Versions
########

Main versions of logic boards and core features:

* FPGA V1 (https://github.com/jhu-cisst/FPGA1394)

  * until to 2015
  * based on FPGA Xilinx Spartan 6
  * two FireWire ports for daisy-chaining
  * See

* FPGA V2 (https://github.com/jhu-cisst/FPGA1394)

  * from 2016 to 2023
  * based on FPGA Xilinx Spartan 6
  * added one Ethernet port (100MB)

* FPGA V3 (https://github.com/jhu-cisst/FPGA1394V3)

  * from 2023
  * based on Xilinx Zynq System on Chip (SoC), XC7Z020
  * embedded dual-core ARM 32 processor
  * added another Ethernet port (two ports, 1GB)

See also the :ref:`controller versions page <controller-versions>` to
determine which FPGA version is used.

.. _nb-fpgas:

Number of FPGAs
###############

The number of FPGAs (and therefore number of FireWire or Ethernet
nodes) depends on the :ref:`controllers versions
<controller-versions>` and the number of arms.  The dVRK Classic
controllers with FPGA V1 or V2 each have two logic boards (aka QLA1).
The dVRK Classic controllers with FPGA V3 (DQLA), the Classic SUJ
controllers (QLA1) and dVRK Si controllers (dRA1) all use a single
logic board.  There is no dVRK-specific FPGA board used for the Si
SUJ.

In practice:

* The initial dVRK kits were composed of two PSMs and two MTMs.  We
  used 4 dVRK Classic controllers so a total of 8 dVRK FPGAs.
* For a full first generation da Vinci with the dVRK controllers V1 or
  V2, we have two MTMs, one ECM, three PSMs and the SUJ.  So six arm
  controllers using 12 logic boards and one more for the Classic SUJ
  controller for a grand total of 13 FPGAs.
* More recent controllers use less FPGA boards so a full Si patient
  cart with two Classic MTMs require only 6 FPGAs.

.. note::

   The expected number of boards is useful when testing :ref:`your
   physical connections <firewire-qladisp>`.
