.. _psm-si:

.. include:: /includes/logic-view-Si-arm.rst

Patient side manipulators
#########################

Arm
===

The da Vinci Si systems came with either 2 or 3 PSMs.  PSM1, PSM2
and PSM3 are identical.

.. figure:: /images/Si/PSM-Si-white-background.png
   :width: 400
   :align: center

   da Vinci Si PSM

Basic information:

* Each PSM arm is attached to the :ref:`SUJ <suj-si>` using 4 long bolts
  and grooves
* The arm can be tilted forward (on Si SUJ; 45 degrees for PSM1 and
  PSM2, 15 degrees for PSM3)
* 7 motorized degrees of freedom, 7 actuators with optical encoders
* All motors are located in the first 2 links.  The first two links
  use a mix of gears and belts.  The last five joints use cable
  transmission
* All actuators have digital potentiometers, used for both homing and
  safety checks
* 3 electric brakes on the first 3 joints to prevent the arm from
  falling when not powered (like the :ref:`ECM Classic <ecm>`)
* The second actuator controls a set of belts used to enforce a RCM
  point using complementary angles (remote center-of-motion, aka
  fulcrum point).  `Google search
  <https://www.google.com/search?q=surgical+robot+center+of+motion+fulcrum+point>`_.
* The kinematic chain is composed of two parts, the arm's base (not
  sterilized) and a removable instrument (sterilized and disposable).
  This design is inherited from the Classic da Vinci so make sure you
  read the section for the :ref:`PSM Classic <psm>`

.. figure:: /images/Si/PSM-Si-white-background-labeled.png
   :width: 600
   :align: center

   da Vinci Si PSM labeled


Cannula and sterile adapter
===========================

The design of the last 4 joints and instrument on the Si system is
very similar to the :ref:`PSM Classic <psm>`.  The main differences are the sterile adapter and cannula
shapes and the cannula holder is attached to the arm.

There is also video at the end of this section describing the process.

.. figure:: /images/Si/PSM-Si-last-4-axis.jpeg
   :width: 250
   :align: center

   Last 4 actuators on PSM Si base arm

.. figure:: /images/Si/PSM-Si-sterile-adapter-installed.jpeg
   :width: 250
   :align: center

   da Vinci Si PSM with sterile adapter (without drape)

One difference between the PSM Classic and Si is that the cannula
holder on the Si is not removable.  The sterile drape comes with a
piece of soft molded plastic that fits between the cannula holder and
the cannula.

.. figure:: /images/Si/PSM-Si-cannula-holder.jpeg
   :width: 300
   :align: center

   da Vinci Si PSM fixed cannula holder

The cannula is reusable and sterilizable.  Cannulas come in different
diameters, 5mm, 8mm... to match the diameter of the shaft of the
instrument used.  Since we don't often use the real sterile drape,
there is a small gap between the cannula holder and the cannula.  To
prevent mechanical, one can use masking tape as a "shim".

.. figure:: /images/Si/PSM-Si-shim-on-cannula.jpeg
   :width: 250
   :align: center

   da Vinci Si PSM 8mm cannula with "shim"

The cannula is held by the cannula holder (shocking) and secured using
two flaps.

.. figure:: /images/Si/PSM-Si-cannula-with-shim.jpeg
   :width: 250
   :align: center

   da Vinci Si PSM cannula in place

.. note::

   Video on YouTube demonstrating how the PSM is prepared:
   https://youtu.be/F7cOVPVq_TY
