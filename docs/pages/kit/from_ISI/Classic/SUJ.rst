.. _suj:

Set Up Joints
#############

On the patient cart, the active arms (two to three :ref:`PSMs <psm>`
and one :ref:`ECM <ecm>`) are mounted on a set of passive arms.  These
passive arms (SUJ) are designed so that each active arm can have its
RCM (remote center of motion) placed at the entry point on the
patient.  Shown below is a model of a Classic patient cart with and
without the active arms.

.. figure:: /images/Classic/SUJ/RViz-Classic-patient-cart.png
   :width: 400
   :align: center

   da Vinci Classic, full patient cart in RViz

.. figure:: /images/Classic/SUJ/RViz-Classic-SUJ.png
   :width: 400
   :align: center

   da Vinci Classic, SUJ only in RViz

Basic information:

* The patient cart is on wheels, so it can be moved in and out the
  operation room
* There are three to four passive arms, one to hold the ECM (SUJ-ECM)
  and two to three to hold the PSMs (SUJ-PSM1, SUJ-PSM2 and optionally
  SUJ-PSM3)
* Each joint has an electric brake.  The brakes are controlled per
  arm.  For example, all the brakes on SUJ-PSM1 are connected to a
  single power source.
* SUJ-PSM1 and SUJ-PSM2 have an extra button to control the brakes on
  the passive arm itself.  All the active arms also have a dedicated
  button to release the brakes, but this signal is handled by the arm
  controller, not the SUJ controller

  .. figure:: /images/Classic/SUJ/SUJ-Classic-PSM12-clutch-button.jpeg
     :width: 350
     :align: center

     da Vinci Classic, clutch/brake button on SUJ for PSM1 and PSM2

* All the passive arms (except PSM-SUJ3) have counterweights in the
  central column, so they can be moved up and down with minimal effort
* The optional SUJ-PSM3 is mounted on front of the column (after
  thought) and doesn't have a vertical counter weight.  Instead, there
  is a lift motor mounted at the base of the cart

  .. figure:: /images/Classic/SUJ/SUJ-Classic-PSM3-lift-motor.jpeg
     :width: 350
     :align: center

     da Vinci Classic, SUJ for PSM3 lift motor

* There is a button attached with a Velcro strap to the active arm
  PSM3.  This button can be toggle up or down to control the height of
  the SUJ-PSM3

  .. figure:: /images/Classic/SUJ/SUJ-Classic-PSM3-lift-button.jpeg
     :width: 350
     :align: center

     da Vinci Classic, SUJ for PSM3 lift button
