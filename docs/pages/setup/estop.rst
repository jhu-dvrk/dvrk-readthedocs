.. _estop:

******
E-Stop
******

Summary
#######

.. figure:: /images/estop/standard-safety-connectors.jpg
   :width: 200
   :align: center

The safety chain, which typically includes an e-stop button, connects
all controller boxes in a configured system. We have standardized on
4-pin and 5-pin safety connectors on all controller boxes, as shown on
the image above.

.. note::

   If your controllers have the 4 and 5 pins connectors, you can just
   read the section :ref:`"Modular E-Stop chain" <estop-modular>` to
   figure out how to use the provided e-stop cables.  Everything else
   is for upgrading older controllers, reference and debugging.

If your controller box does not already have these connectors, as of
June 2018, we have developed retrofit kits to convert earlier
controller boxes to this standard. Designs, documentation and images
of the retrofit kits can be found in this `GitHub project
<https://github.com/jhu-dvrk/dvrk-estop-retrofit>`_ and assembled
retrofit kits (i.e., small PCBs with connectors) are being made
available to the community at no cost.

Overview of safety chain
########################

Each controller box contains one or two connectors to support a
"safety chain". The basic concept is that the safety chain must be
unbroken (i.e., electrically closed) for the motor power supplies in
all of the connected controller boxes to be enabled.  If any device on
the safety chain breaks electrical continuity, then all motor power
supplies in all connected controller boxes are disabled.  Devices in
the safety chain include manually operated switches, such as the
e-stop button, and computer-controlled relays, which enable the system
to disable motor power. This is illustrated in Figure 1, which shows
the safety wiring in a controller box. In particular, each controller
box contains 3 relays:

* 1 Safety Relay on each of the two QLA boards
* 1 relay to control the motor power supply (AC Input Relay)

To enable the motor power supply, the 12V power needs to be connected
to the AC Input Relay.

.. figure:: /images/estop/estop_relay.jpg
   :width: 500
   :align: center

   Figure 1: Controller box relays and power supply

Figure 1 shows the standard safety connectors (one 4-pin and one
5-pin) on the controller box. There have, however, been several
variations of safety connectors in the different generations of
controller boxes, as shown in the figure below. All of them can be
upgraded to the standard configuration (shown on the right) by using
the appropriate `retrofit kit
<https://github.com/jhu-dvrk/dvrk-estop-retrofit>`_

.. figure:: /images/estop/estop_connectors.png
   :width: 600
   :align: center

   Figure 2: Safety connectors for the different generations of
   controller boxes. Standard configuration (one 4-pin and one 5-pin
   connector) shown on right.

The 5-pin connector is the same as the 4-pin connector except that pin
#2 is GND, which shifts the signals from pins 2-4 on the old (4-pin)
connector to pins 3-5 on the new (5-pin) connector. Note also that in
the final design, pin #1 of the 4-pin connector is GND rather than
12V.

The advantage of the final design, with one 4-pin and one 5-pin
connector, is that it enables both a :ref:`modular (reconfigurable)
e-stop chain <estop-modular>`, developed at JHU and currently used on
most systems, and a :ref:`monolithic (hard-wired) e-stop chain
<estop-monolithic>`, as initially implemented at WPI.  Note that this
was the intended goal of the systems with two 5-pin connectors, but in
those systems it is possible to accidentally bypass some of the safety
relays, as described in :ref:`this section below
<estop-issue-2-5-pins>`.

.. _estop-modular:

Modular E-stop chain (recommended)
##################################

The modular e-stop chain, also called reconfigurable e-stop chain, consists of three different types of cables:

1. E-Stop Cable: connects the e-stop to the 5-pin connector on one
   controller box
2. Extension (Daisy-Chain) Cable: connects the 4-pin connector on one
   box to the 5-pin connector on another box
3. Termination Plug: placed on a 4-pin connector on one of the
   controller boxes

These are shown in the following image (from the dvrk-estop-retrofit project):

.. figure:: /images/estop/dvrk-estop.png
   :width: 400
   :align: center

This design is intended to enable quick reconfiguration of the safety
circuit. For example, a complete DVRK setup (4 daisy-chained
controller boxes) would have 1 E-Stop Cable, 3 Extension Cables, and 1
Termination Plug. To split this into two separate systems (e.g.,
MTMR+PSM1 and MTML+PSM2), each system would use 1 E-Stop Cable, 1
Extension Cable, and 1 Termination Plug.

.. _estop-monolithic:

Monolithic E-stop chain (not recommended)
#########################################

The monolithic e-stop chain, previously called the serial e-stop
chain, is built to connect to a specific number of controllers. For
example, the figures below show the connection to two controller boxes
(left) and to four controller boxes (right). Both examples show
controllers with 4-pin connectors. If the controller box has a 5-pin
connector, it would be better to use that, so that the GND can also be
connected.

This setup is a natural fit for controller boxes with only one safety
connector and can be used for controller boxes with two connectors (in
which case the 5-pin connector would be used), but has the
disadvantage that the cable must be redone to support fewer or more
controllers. Thus, for controller boxes with a single connector, we
recommend installing the `retrofit kit
<https://github.com/jhu-dvrk/dvrk-estop-retrofit>`_ to obtain the
standard two connector configuration and instead use the :ref:`modular
E-stop chain <estop-modular>`.

.. figure:: /images/estop/daVinci-Estop_2Controllers-1.png
   :width: 400
   :align: center

   Two controller boxes (not recommended)

.. figure:: /images/estop/daVinci-Estop_4Controllers-1.png
   :width: 400
   :align: center

   Four controller boxes (also not recommended)

.. _estop-issue-2-5-pins:

Issue with two 5-pin connectors
###############################

The Build #4 controller boxes contain two 5-pin safety
connectors. While the intent was to enable both the modular and
monolithic connection schemes, it is possible to miswire the e-stop
chain because the Extension (Daisy-Chain) Cable is not
straight-through; it connects S1 (pin 3) on one connector to S2
(pin 4) on the other. Thus, it matters which way the cable is
connected. Connecting it backwards will cause some of the safety
relays to be bypassed, as shown in the figure below.

.. figure:: /images/estop/SafetyChain-Build4.png
   :width: 600
   :align: center

A quick note about grounding
############################

It is good practice to connect the GND (ground) on all components of a
system. The original controller box, with a 4-pin safety connector,
did not include a specific GND connection between controller boxes and
therefore relied on the likelihood that the GND would be shared via
the AC wiring (e.g., if all controller boxes are plugged in to the
same AC circuit).

To correct this deficiency, later versions of the controller box
include at least one 5-pin safety connector, where the extra pin is
GND. This provides a way to guarantee that GND will be shared between
all controller boxes in a system (where the "system" is defined by
which boxes are connected via the safety chain).

Since the 4-pin safety connector does not include a GND pin, this GND
connection could be obtained by attaching via a screw on the
enclosure. See the notes on the `dvrk-estop-retrofit project
<https://github.com/jhu-dvrk/dvrk-estop-retrofit>`_. On the 5-pin
safety connector, the GND is available on pin 2.

Debugging
#########

If you have trouble powering on the motors, please continue reading this section.

Test single FPGA-QLA board set (bypassing relays on QLA boards)
***************************************************************

As step 1, we want to confirm that the FPGA board, QLA board and power
supplies all work. We do this by bypassing the internal relays in the
box but keeping the E-STOP in the chain as shown in the next
figure. This is done by connecting the +12V to the EN (enable) signal,
via the e-stop. The figure below shows the wiring with the 5-pin
safety connector, but similar wiring can be done with the single 4-pin
connector.

Connect the modified connector to the controller box you want to debug and run the ''qladisp'' program: 

.. code-block:: bash
		
   # assume we are testing MTML box 
   $ qladisp 0 1  

   # Press 'p' to turn on power
   #   - 'p' first turns on the QLA relays (this step does NOT matter, since those relays are bypassed)
   #   - then turns on board and amplifier power
   # The mv-good and all amplifiers should be turned on at this time 
   #   - if not, check the power system physical connections


.. figure:: /images/estop/estop_bypass_one.jpg
   :width: 600
   :align: center

.. _estop-test-single:

Test single controller box with QLA relays in the loop
******************************************************

After confirming that the power system is working, we start to add
relays inside one controller box to the chain.  With the [modular
connector setup](#3-modular-e-stop-chain-recommended), you should
attach the E-Stop cable to the 5-pin connector and the Termination
Plug to the 4-pin connector, as shown in the following figure.
Alternatively, for a single 4-pin or 5-pin connector, modify the cable
to attach 12V to S1 and S2 to EN.

.. code-block:: bash
		
   # assume we are testing MTML box
   # relays are serial chained, so connect to 0 and 1 at the same time  
   $ qladisp 0 1

   # Press 'p' to turn on power
   #   - 'p' first turns on the QLA relays
   #   - then turns on board and amplifier power
   # The mv-good and all amplifiers should be turned on at this time
   #   - if not, check the Hardware Debug section (below)

.. figure:: /images/estop/estop_one.jpg
   :width: 600
   :align: center

Hardware Debug
**************

I'm sorry you are reading this section, but we need to figure it
out. You will need a multimeter to debug.

The very first step is to check if the relay on the QLA board is
working. As shown in the following figure, there are two test points
(T1, T2). NOTE, the relay might look different depending on your
hardware revision. The connection between the two points is designed
to be open when the relay is turned OFF and shorted when the relay is
ON.

Assume we test board 0 first:

* Do a continuity test between T1 and T2. It should be open; if not, contact us. (No power)
  
* Turn on relay now

 * ``$ qladisp 0``
 * Press 'p' to turn on relay
 * You should also hear a click sound from the relay

* Do a continuity test between T1 and T2. Now they should be shorted;
  if not, contact us.

REPEAT the same process for board 1.

.. figure:: /images/estop/estop_relay_debug.png
   :width: 600
   :align: center

Now, you have two working relays. Please check the wire connection,
make sure:

1. they are serially connected
2. the connection to the E-STOP terminal is correct.

The next step is to test them together. The S1 and S2 pins of the
safety connector are connected to the two relays. If the system is
working, they should be open while the relays are OFF and shorted
while the relays are ON.

Assume we are testing MTML board 0 and 1:

* Do a continuity test between S1 and S2. Should be open.
* Turn on relay now

  * ``$ qladisp 0 1``
  * Press 'p' to turn on relay
  * You should also hear a click sound from the relay

* Do a continuity test between S1 and S2. Now they should be shorted; if not, check the wire connection.

Finally, do the test in :ref:`"Test single controller box with QLA
relays in the loop" <estop-test-single>`. You should be good to go.
