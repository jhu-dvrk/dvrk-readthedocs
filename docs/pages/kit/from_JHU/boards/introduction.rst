************
Introduction
************

The overall design of the dVRK controllers is based on centralized computing (on
the PC) and distributed I/O (over FireWire or Ethernet).  Furthermore, the
controllers use two distinct components, a logic board (also known as FPGA or
FPGA1394) and a board for signal processing and motor/brake power: QLA for da
Vinci Classic and dRAC for da Vinci Si.  The custom electronic components (FPGA,
QLA and dRAC) are all designed by the Johns Hopkins group under the direction of
Dr. Peter Kazanzides.  The board designs, bills of materials, and all the
firmware source is freely available on GitHub.

Different boards are used on different configuration and generations of dVRK
controllers.

.. figure:: /images/controllers/dVRK-signals-all-controllers.*
   :width: 600
   :align: center

   Overall view of dVRK controllers integration

All acronyms are defined in the :ref:`reference section <acronyms>`.  The
following sections provide an overview of the main dVRK custom boards: FPGA, QLA
and dRAC.
