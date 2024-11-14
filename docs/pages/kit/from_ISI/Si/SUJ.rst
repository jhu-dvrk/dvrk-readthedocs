.. _suj-si:

Set Up Joints
#############

See :ref":`SUJ Classic <suj>` for overview.

.. figure:: /images/Si/RViz-Si-SUJ.png
   :width: 400
   :align: center

   da Vinci Si, SUJ in RViz

Basic information:

* The patient cart is on wheels and is motorized, so
  it can be moved in and out the operation room
* There are four passive arms, one to hold the ECM (SUJ-ECM) and three
  to hold the PSMs (SUJ-PSM1, SUJ-PSM2 and SUJ-PSM3).  A two PSMs
  model was also available, the da Vinci Si-E (no PSM3)
* Each joint has an electric brake.  The brakes are controlled per
  arm.  For example, all the brakes on SUJ-PSM1 are connected to a
  single power source.
* The passive arm SUJ-PSM3 can be positioned on either side of the
  patient cart.  The SUJ-PSM3 has latches on the back of the cart, one
  for the left position and one for the right position.  In the
  pictures below, the SUJ PSM3 is coming on the left of the cart

  .. figure:: /images/Si/SUJ-Si-latch-PSM3-not-used.jpeg
     :width: 300
     :align: center

     da Vinci SUJ PSM3, right locking mechanism not used

  .. figure:: /images/Si/SUJ-Si-latch-PSM3-used.jpeg
     :width: 300
     :align: center

     da Vinci SUJ PSM3, left locking mechanism used

* There are no buttons to control the brakes on the SUJ, all the
  buttons are on the active arms
* Each SUJ passive arm has a small board with analog-to-digital
  converters to read the joint potentiometers (ESSJ)
