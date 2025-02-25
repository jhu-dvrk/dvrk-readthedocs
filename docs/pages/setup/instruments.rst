.. _setup-instruments:

***********
Instruments
***********

Some instruments are supported by the dVRK but need to be physically
modified.  For example, Permanent Cautery Spatula (xxx184):

We need to "disable" disk #4 on the instrument because the actuator 3
and 4 are mechanically coupled inside the instrument.  At that point
(December 2019), the dVRK software doesn't support driving 2 actuators
coupled together to control a single joint.  Since it is hard to
remove the mechanical coupling inside the instrument, we recommend to
disable the disk #4 (either by removing the disk or filling off the
plots on the disk).

.. figure:: /images/instruments/permanent-cautery-spatula/permanent-cautery-spatula.jpg
   :width: 350
   :align: center

   Permanent cautery spatula

.. figure:: /images/instruments/permanent-cautery-spatula/permanent-cautery-spatula-coupling.jpg
   :width: 450
   :align: center

   Permanent cautery spatula mechanical coupling

.. figure:: /images/instruments/permanent-cautery-spatula/permanent-cautery-spatula-disks.jpg
   :width: 350
   :align: center

   Permanent cautery spatula mechanical coupling

.. figure:: /images/instruments/permanent-cautery-spatula/permanent-cautery-spatula-modifying.jpg
   :width: 350
   :align: center

   Permanent cautery spatula modification

.. figure:: /images/instruments/permanent-cautery-spatula/permanent-cautery-spatula-modified.jpg
   :width: 350
   :align: center

   Permanent cautery spatula modified
