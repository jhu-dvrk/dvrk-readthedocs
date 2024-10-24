.. _si-internal:

Internal layout
###############

The FPGA V3/dRAC-based controllers contain one FPGA V3 and one dRAC
board, a LED board, 2 power supplies (12V logic and 48V motors and
brakes).  The FPGA is mounted against the controller's front panel so
we can use the FireWire and Ethernet ports directly.  The dRAC is
mounted against the back of the controllers to expose the 2 D-sub and
e-stop connectors.  This design greatly reduces the amount of internal
wiring compared to the dVRK Classic controllers.

.. figure:: /images/controllers/drac-controller-diagram.png
   :width: 600
   :align: center

   dRAC, FPGA V3 based Si controller internals
