SUJ
===

Preparation
-----------

Remove everything from patient cart

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

The dESSJ boards should come pre-programmed. If they're not programmed
or you need to upgrade the firmware, please see
https://github.com/jhu-dvrk/dESSJ-firmware

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
   :width: 400
   :align: center

   Label on each active Si arm
