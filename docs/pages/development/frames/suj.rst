.. _frames_SUJ:

SUJ
###

Overview
********

When setup joints are present, the following positions are reported:

* ECM frames:

  * ECM-SUJ-Local:

    * Defined with respect to patient cart origin
    * Uses ``DH * tooltip-offset`` from ``suj.json``
    * Defines ECM-RCM

  * ECM-SUJ:

    * Same as ECM-SUJ-Local
  * ECM-Local:

    * Defined with respect to the ECM RCM
    * Uses ``DH * tooltip-offset`` from ``share/kinematics/ecm.json``
    * ``tooltip-offset`` depends on type of scope (looking straight,
      up or down)

  * ECM:
    
    * Camera frame defined with respect to **patient cart**
    * Uses ``ECM-SUJ * ECM-Local`` (``ECM-SUJ`` is the base frame)

* PSM frames

  * PSM-SUJ-Local:

    * See ECM-SUJ-Local

  * PSM-SUJ:

    * Defined with respect to camera frame
    * Uses ``inverse(ECM) * PSM-SUJ-Local`` (``inverse(ECM)`` is the
      base frame)

  * PSM-Local:

    * See ECM-Local
    * ``DH``, ``tooltip-offset``... depend on type of instrument
      (found in ``share/tool``)

  * PSM:

    * PSM tooltip frame defined with respect to **camera frame**
    * Uses ``PSM-SUJ * PSM-Local`` (``PSM-SUJ`` is the base frame)


The following assumes the ECM is the reference arm for all the PSMs.
Versions greater than 2.1, one can change the reference arm in the SUJ
configuration file. ⊙ represents the reference frame and the arrows
show the transformations.

.. csv-table:: dVRK transformation compositions and topics
   :name: dvrk-suj-frames
   :header: "Command", "PSM tip", "PSM RCM", "Cart", "ECM RCM", "ECM tip"
   :align: center

   " ``ECM/measured_cp`` ",          "  ", "  ", "⊙", "➡", "➡"
   " ``ECM/local/measured_cp`` ",    "  ", "  ", "  ", "⊙", "➡"
   " ``SUJ/ECM/measured_cp`` ",      "  ", "  ", "⊙", "➡", "  "
   " ``SUJ/ECM/local/measured_cp``", "  ", "  ", "⊙", "➡", "  "
   " ``PSM/measured_cp``",           "⬅", "⬅", "⬅", "⬅", "⊙"
   " ``PSM/local/measured_cp``",     "⬅", "⊙", "  ", "  ", "  "
   " ``SUJ/PSM/measured_cp``",       "  ", "⬅", "⬅", "⬅", "⊙"
   " ``SUJ/PSM/local/measured_cp``", "  ", "⬅", "⊙", "  ", "  "

Intuitive SUJ
*************

The following is the graph of transformations for a system that uses a
dVRK SUJ controller.  These controllers (on Classic or Si) can read
the potentiometers of the SUJ and therefore allow us to compute the
forward kinematics of each SUJ arm.

.. figure:: /images/transformations/dVRK-transformations-SUJ.png
   :width: 600
   :align: center

   Patient's side with SUJ dVRK controllers

SUJ Fixed
*********

For groups that don't have access to a SUJ controller but have an ECM,
we recommend to use the SUJ Fixed class.  This class maintains the
transformation graph one would get with a real SUJ but the SUJ
transformations have to be provided by the user.  These
transformations are usually the result of a registration.  Using the
SUJ Fixed class allows user to control the PSMs with respect to the
ECM even when the ECM moves (e.g. ECM teleoperation).

.. figure:: /images/transformations/dVRK-transformations-SUJ-Fixed.png
   :width: 600
   :align: center

   Patient's side with SUJ without dVRK controllers (SUJ Fixed)

The class SUJ Fixed doesn't make any assumption regarding the main
reference frame (aka "World").  Users could use an optical tracking
system and find the respective base frame of the PSMs and ECM with
respect to the tracking system.  In this case, the result of the
registration represent ``/SUJ/PSM{1,2,3}/local/measured_cp`` and
``/SUJ/ECM/local/measured_cp``.

If the registration method is an hand-eye registration with respect to
the ECM base (aka ECM RCM), the transformation for
``/SUJ/ECM/local/measured_cp`` is identity and ``ECM RCM`` is the same
as `World` (see https://github.com/jhu-dvrk/dvrk_camera_registration).

.. figure:: /images/transformations/dVRK-transformations-SUJ-Fixed-hand-eye-calibration.png
   :width: 600
   :align: center

   Patient's side with SUJ Fixed, hand-eye registration
