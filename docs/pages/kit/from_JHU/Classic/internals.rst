.. _classic-internal:

Internal layout
###############

FPGA V1/V2 based
****************

Internally, each V1 or V2 QLA1-based controller box contains two
FPGA/QLA board sets, one dMIB (da Vinci Manipulator Interface Board),
LED boards, power supplies and relays.

.. figure:: /images/controllers/qla1-controller-diagram.png
   :width: 600
   :align: center

   QLA, FPGA V1 based Classic controller layout

.. figure:: /images/controllers/qla1-controller-layout.jpg
   :width: 600
   :align: center

   QLA, FPGA V1 based Classic controller internals

FPGA V3 based
*************

The V3 DQLA1-based controllers contain one FPGA V3 and 2 QLA boards, a
set of DQLA-Q/DQLA-F (connected using 2 flat ribbons), one dMIB (da
Vinci Manipulator Interface Board), LED boards, power supplies and
relays.  The FPGA is mounted against the controller's back panel so we
can use the FireWire and Ethernet ports directly.  Compared to the
QLA1-based controllers, this greatly reduced the amount of internal
wiring (Ethernet and FireWire pass-through cables).

.. figure:: /images/controllers/dqla-controller-diagram.png
   :width: 600
   :align: center

   DQLA, FPGA V# based Classic controller layout

.. figure:: /images/controllers/dqla-controller-layout.png
   :width: 600
   :align: center

   DQLA, FPGA V3 based Classic controller internals
