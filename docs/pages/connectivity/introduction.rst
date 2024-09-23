************
Introduction
************

There are different ways to connect the dVRK controllers depending on
which version of dVRK controllers are used.

While testing the communication with the dVRK controllers, keep in
mind that the number of FPGA boards (showing as either a FireWire node
or and Ethernet IP) depends on the type of controller you're using and
how they are connected.  See :ref:`controllers<nb-fpgas>`.

FireWire only
=============

From the beginning, the dVRK software and mechatronics have used the
IEEE 1394 interface standard with the `libraw1394
<http://www.dennedy.org/libraw1394/>`_ library under Linux to
communicate between the computer and the dVRK controllers. The
interface standard IEEE 1394 is also known as FireWire (`Wikipedia
entry <https://en.wikipedia.org/wiki/IEEE_1394>`_). In this scenario,
the controllers are daisy-chained using FireWire and the computer is
also on the FireWire chain.  This approach works will all generations
of dVRK controllers and is the most reliable approach.

.. figure:: /images/connectivity/PC-FireWire-Controllers.png
   :width: 400
   :align: center

   FireWire only

FireWire with Ethernet bridge
=============================

Starting with dVRK Software Version 2.0, Ethernet/UDP is also
supported. To use Ethernet/UDP, you will need to use firmware 7+ on
all your dVRK controllers and you will need at least one FPGA V2.x
board (with Ethernet jack, see :ref:`FPGA versions<fpga-version>`). In
this scenario, the controllers are daisy-chained using FireWire but
the computer is not on the FireWire chain. Instead, the computer is
connected via Ethernet to one of the dVRK controllers. Said dVRK
controller becomes the "bridge" between the computer and all the other
dVRK controllers.

.. figure:: /images/connectivity/PC-Ethernet-Controllers.png
   :width: 400
   :align: center

   FireWire with Ethernet bridge

Ethernet only
=============

todo

Bluetooth
=========

For the dVRK Si, the communication between the SUJ and the PC relies
on Bluetooth.  This was the cheapest and fastest solution to read the
SUJ potentiometers without adding external wires.  This solution works
fine but is a bit tricky to setup.
