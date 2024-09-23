SUJ
===

Preparation
-----------

The goal is to remove most of the electronics from the back of the
patient cart.  This is required since the cables coming from the base
of the column of the cart are fairly short and need to reach the back
of the dVRK controllers.

.. warning::

   Most of the cables we need to reconnect to the dVRK controllers are
   plugged on the 4 RACs, on the front of the patient cart.  There is
   one lone exception, a back 2-wires cable.  Make sure you keep track
   of this cable.

.. warning::

   Once you've setup the Si cart for the dVRK controllers, it's
   possible but really difficult to go back to the original
   controllers.

The first step is to remove the cover (2 parts) on the back of the
cart.  The cover hold with 2 screws and some pressure clips.

.. figure:: /images/Si/Si-SUJ-conversion-top-screws.png
   :width: 400
   :align: center

   SUJ Si top cover screws

Once the screws are removed, you will need to pry out the covers.  The
pressure clips are fairly strong so don't be afraid to apply some force
on the cover.

.. figure:: /images/Si/Si-SUJ-conversion-top-cover.png
   :width: 300
   :align: center

   SUJ Si top cover

Once the covers are removed, you should familiarize yourself with the
different cables you will need to unplug from the original electronics
and re-plug to the dVRK controllers.  Ultimately, the RACs, back
panels, bracket, cover, battery, electronics... will be removed.

.. figure:: /images/Si/Si-SUJ-conversion-top-open.png
   :width: 400
   :align: center

   SUJ Si open, top view

.. figure:: /images/Si/Si-SUJ-conversion-front-open.png
   :width: 400
   :align: center

   SUJ Si open, front view

On the top view, you can find the "Z-AXIS" connector. Make sure you
squeeze the tab to unplug it.  Keep that cable aside. it will be
connected to the 5th dESSJ.

On the front view, you can see 4 RACs, i.e. one per active arm.  They
are labeled SJA2, SJX, SJC and SJA1.  These cables will need to be
plugged in the dSIB for the PSM2, PSM3, ECM and PSM1 respectively.

See next section for the :ref:`dESSJ<dessj-setup>` and
:ref:`dSIB<dsib-si-setup>` descriptions.

.. figure:: /images/Si/Si-SUJ-conversion-Z-axis-dESSJ.png
   :width: 500
   :align: center

   SUJ Si Z-Axis and mapping to dESSJ

On the front view, you can locate the 4 original RACs as well as all
the connectors that will be connected to the dSIB on the dVRK
controllers.  There are 2 connectors from a bundle (white), a 2-wires
black cable, a FireWire B cable (aka LVDS) and a grounding cable with
a ring terminal.  Make sure all these cables are labeled.

.. figure:: /images/Si/Si-SUJ-conversion-RAC-cables-labels.png
   :width: 300
   :align: center

   SUJ Si active arm bundle

.. caution::

   The FireWire cable is mounted on the back of the dRAC PCB, label
   J11. There is not much space to unplug it. We strongly recommend
   you detach the RAC (the whole bloc) from the large steel plate
   holding all 4 RACs so you can unplug the cable without damaging the
   connector.

.. figure:: /images/Si/Si-SUJ-conversion-RAC-connectors.png
   :width: 500
   :align: center

   SUJ Si RAC connections

Once you've disconnected of the cables we need to preserve for the
dVRK, you can remove all the electronic, bracket, battery... from the
patient cart.  Ideally there should be noting above the "shoulders" so
you have a somewhat flat surface to stack the 4 dVRK Si controllers.

.. figure:: /images/Si/Si-SUJ-conversion-RAC-mapping.png
   :width: 500
   :align: center

   SUJ Si mapping from RAC to dSIB

.. _dessj-setup:

dESSJ
-----

The dESSJ boards are custom boards designed to replace the ESSJ in the
SUJ itself.

.. note::

   Do not trash the existing ESSJ boards, we might be able to provide
   support for these later on. They have better analog to digital
   converters than the Arduino based dESSJ.

Four ESSJ are located on the SUJ arm, under a small metal cover. For
the dVRK, we use a fifth board at the base of the SUJ for the linear
potentiometers located in the patient cart's column. The 4 replacement
boards in the SUJ arms are pass-through for the LVDS communication
between the arm and the controller (FireWire B physical connector, not
a real FireWire port). For the SUJ's analog potentiometers, we use an
Arduino with Bluetooth Low Energy (BLE) to communicate with the PC.

.. warning::

   For the Bluetooth connection to work, do not put the metal covers
   back. You will also need to run a long USB3 cable close to the SUJ
   and plug a Bluetooth adapter since the BLE signal is pretty weak.

There are 3 different uses for the dESSJ:

 * For the ECM, PSM1 and PSM2: use both LVDS connectors and 3
   potentiometer connectors (J4, J5 and J6) as well as power
 * For the PSM3: use both LVDS connectors and 4 potentiometer
   connectors (J4, J5, J6 and J7) as well as power
 * For the 5th dESSJ: there is no LVDS but you need to connect the "z
   axis only" to the potentiometer cable from the SUJ column. The
   board must be powered using a micro USB connected to the Arduino
   board on the dESSJ.  We don't manufacture an enclosure for the 5th
   dESSJ so make sure you wrap it in a plastic bag to avoid short
   circuits.

.. figure:: /images/Si/Si-SUJ-conversion-ESSJ-mapping.png
   :width: 400
   :align: center

   SUJ Si mapping from ESSJ to dESSJ

.. note::

   Since the 5th dESSJ doesn't need FireWire connectors, it is usually
   shipped without them.

The dESSJ boards should come pre-programmed. If they're not programmed
or you need to upgrade the firmware, please see
https://github.com/jhu-dvrk/dESSJ-firmware

.. _dsib-si-setup:

dSIB-Si
-------

For a full system, you will need 4 dSIB adapter boards, one for each
dVRK-Si controller (da Vinci Setup Interface Board). These boards are
plugged between the dVRK Si controllers and the original cables coming
at the base of the SUJ column. These boards allow us to re-use the
internal cables in the SUJ arms. The cables coming from the column are
unfortunately very short so you will have to remove the cover on the
back of the SUJ as well as most of the existing electronics at the
base of the patient cart. The dSIB boards are also used to power the
SUJ brakes.

JHU setup
---------

The following images show the dVRK Si at Johns Hopkins.  We strongly
recommend to put labels everywhere to avoid confusing cables,
controllers and boards. The labels should have the arm name and, when
needed, the arm serial number as well as the MAC address of the dESSJ
board.

.. figure:: /images/Si/SUJ-dESSJ-clear-cover-labeled.jpg
   :width: 400
   :align: center

   dESSJ on SUJ PSM3 (bottom) and ECM (top)

.. figure:: /images/Si/Si-controllers-SUJ-front-labeled.jpg
   :width: 400
   :align: center

   Stack of dVRK-Si controllers with data cables (front)

.. figure:: /images/Si/Si-controllers-SUJ-back-labeled.jpg
   :width: 400
   :align: center

   Stack of dVRK-Si controllers with dSIB-Si (back)

.. figure:: /images/Si/PSM-Si-label-labeled.jpg
   :width: 300
   :align: center

   Label on each active Si arm
