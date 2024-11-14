.. _instruments:

***********
Instruments
***********

Introduction
############

On the patient's side, the PSM arms are designed to use
interchangeable instruments depending on the task (e.g. needle driver,
scissor, grasper...).  These instruments are sterilizable and
disposible.  Each instrument has a maximum number of uses based on its
mechanical properties and expected usage.  After multiple uses (and
required sterilization between uses), the internal mechanisms are more
likely to break.  This can lead to injuries, hence a set maximum
number of "lives" per instrument.

.. figure:: /images/instruments/instrument-Classic-8mm-vs-5mm-labeled.jpg
   :align: center

   Examples of da Vinci Classic instruments, 8 and 5 mm diameter

The main parts of a da Vinci instrument are:

* disc plate, back of the instrument with 4 discs that will mate with
  the sterile adapter and the actuators
* housing, plastic cover for the mechanisms used to transmit the
  motion from the actuators to the joints

  .. figure:: /images/instruments/instrument-internal.jpeg
     :width: 250
     :align: center

     Instrument's base

* two release levers, used to remove the instrument from the sterile
  adapter
* rotating shaft, long tube containing cables/rodes to transmit the
  motion for the last 3 actuators
* an articulated wrist with 2 degrees of freedom (EndoWrist)
* a tip, articulated (scissor, grasper, forceps...) or not (hook,
  spatula...)

.. note::

   Catalogs of instruments can be found on the web or in the
   :ref:`References - Links - Intuitive Surgical section
   <links-isi-docs>`.

Part number
###########

Instruments can be identified using their part number:

* First 6 digits: base part number
* Dash '-'
* Last 2 digits: version number
* Optional "T" letter: training instrument, same mechanical parameters
  but can be used more times (on clinical systems, irrelevant on dVRK)
* Letter Revision: 'A-Z'

Example: 420093-03T Rev B

Classic vs S/Si
###############

The main differences between the first generation of instruments
(Classic) and the second (S and Si) are:

* Model number: both generation of instruments are identified by a 6 digits
  number.  Classic instrument model numbers start with **400** while S/Si
  instrument model numbers start with **420**.  For example, a classic large
  needle driver (LND) uses **400**\ 006 while a S/Si LND uses
  **420**\ 006.
* Length: S/Si instruments are longer.  If you are curious, you should
  look at S/Si PSM arms, they have an interesting two stage
  translation link for insertion.  This allows to "collapse" the arm
  when the instrument is inserted and reduce the risk of collision
  between the arms outside the patient.  For example, the Classic LND
  has a shaft that measures 41.62 cm while the S/Si LND has a 46.7 cm
  shaft (5.08 cm longer).
* Shape of the instrument disk plate and sterile adapter: since both
  the Classic and S/Si were commercially available at the same time,
  the instruments have a different base plate to make sure a Classic
  instrument can't be accidentally inserted on a S/Si PSM and a S/Si
  instrument can't be inserted on a Classic PSM.

.. figure:: /images/instruments/instrument-Classic-vs-Si-housing.jpeg
   :width: 400
   :align: center

   Housing differences between Classic (left) and S/Si instruments (right)

   .. figure:: /images/instruments/instrument-Classic-vs-Si-disc-face.jpeg
      :width: 400
      :align: center

   Disc plate differences between Classic (left) and S/Si instruments (right)

.. note::

   Since the da Vinci classic has been retired in 2012, it is getting
   harder to find instruments for this model.  Fortunately, when
   Intuitive Surgical introduced the da Vinci S (and later Si) the
   instruments remained **almost** unchanged so it is possible to use
   S/Si instruments with the dVRK Classic PSMs with a :ref:`modified
   Classic sterile adapter <classic-adapter-si>`.  Classic instruments
   can not be used on the dVRK Si.

EndoWrist
#########

The wrist design is pretty much the same between the Classic and Si
instruments.  Instruments actually share their names and serial's
number last 3 digits across generations.  A Large Needle Driver for
the da Vinci Classic has the serial number 400\ **006** while the
Large Needle Driver for the da Vinci S and Si has the serial number 4\
**2**\ 0\ **006**.

The following pictures show different wrist mechanisms.

.. figure:: /images/instruments/debakey-EndoWrist.jpeg
   :width: 250
   :align: center

   da Vinci DeBakey grasper wrist and tip

For the smaller diameter instruments, the wrist is composed of 4
stacked disks.  This is sometimes known as a snake-like robot.

.. figure:: /images/instruments/needle-driver-5mm-EndoWrist.jpeg
   :width: 250
   :align: center

   da Vinci 5 mm instrument wrist and tip

For energized instruments (see next section), the wrist has to include
some electric insulators in the last joint. For a bipolar instrument,
each jaw is isolated from the other and the rest of the instrument

.. figure:: /images/instruments/instrument-bipolar-EndoWrist.jpeg
   :width: 250
   :align: center

   da Vinci bipolar wrist and tip

Energy
######

Some da Vinci instruments can be used for cauterization.  There are
two categories of energized instruments, monopolar and bipolar.
Monopolar cauterization instruments can easily be identified by the
single power pin that comes from the housing.

.. figure:: /images/instruments/instrument-monopolar-housing.jpeg
   :width: 250
   :align: center

   da Vinci Classic monopolar instrument, housing and connector

On the other hand, bipolar instruments have a connector with 2 pins.

.. figure:: /images/instruments/instrument-bipolar-housing.jpeg
   :width: 250
   :align: center

   da Vinci Classic bipolar instrument, housing and connector


Newer Si instruments
####################

ISI introduced a new mechanism for the instrument's roll sometime
around 2020.  Instead of using a cable, the roll is now controlled
using a gear.  This affects the coupling matrix used to convert
actuator to joint values.  The scaling factor is different and the
direction is reversed.  It is very important to make sure you have the
correct instrument definition file.  You can check if you have a
"geared" instrument by looking at the mating disks or the revision
number on the cover.

.. figure:: /images/instruments/LND-S-rev12.jpg
   :width: 400
   :align: center

   Large needle drivers with "traditional" cable-based and "new"
   gear-based roll mechanism

.. figure:: /images/instruments/LND-Si-rev12-label.jpeg
   :width: 250
   :align: center

   Large needle with "new" gear-based roll mechanism ("VER 12")

.. warning::

   If you are using a "geared" instrument and the cartesian motion
   doesn't make any sense, it might be because the dVRK developpers
   are not aware of this new revision.  Let us know!
