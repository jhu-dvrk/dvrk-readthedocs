.. _davinci-si:

Introduction
############

History
=======

The second generation (S) is in between the first and third.  It
shares most of the parts and designs from the Classic for the
surgeon's console and video tower.  The patient cart was completely
redesigned for the da Vinci S.  The Si patient cart mostly carries
over the S patient cart (see :ref:`da Vinci generations
<davinci-generations>`).

The third generation of da Vinci systems (Si) share very few features
with the first generation (Classic) since the surgeon's console is new
and the patient's cart is basically an S patient's cart.  The few
notable exception are:

* The instruments geometry, albeit the Si instruments are slightly
  longer. We can use Si instruments on the dVRK Classic PSMs.
* ECM endoscope adapter. This means that we can use any endoscope
  coming from a Classic, S or Si on an ECM Classic, S or Si.

.. note::

   The da Vinci Si surgeon's console has been carried over to the Xi
   systems (4th generation).  Since the Xi systems are not retired
   yet, these consoles are not available for the research community
   (as of 2024).  On the other hand, availability of the patient's
   cart for the S and Si should increase as the Si systems are being
   retired (by end of 2024).

Main components
===============

.. figure:: /images/general/ISI-da-Vinci-S-overview-labeled.png
   :width: 600
   :align: center

   da Vinci S: surgeon's console, patient's cart and video tower

.. figure:: /images/general/ISI-da-Vinci-Si-overview-labeled.png
   :width: 600
   :align: center

   da Vinci Si: surgeon's console, patient's cart and video tower

dVRK integration
=================

The following components can be used for the dVRK (see :ref:`dVRK
setup <setup-si>`)

* Surgeon's console for S systems, see :ref:`da Vinci Classic <davinci-classic>`.

* Video tower (S and Si), see :ref:`dVRK video support <video>`

  * Light source
  * Camera control units
  * Focus controller (for S systems only)
  * The rack itself if provided along the dVRK

* Patient's cart

  * SUJ: setup joints, cart with central column and up to 4 passive
    arms with brakes to position the active arms around the patient (S and Si)
  * PSMs: patient side manipulators, active arms holding the
    laparoscopic instruments with EndoWrist (Si and some S, see
    :ref:`generation support <davinci-generations>`)
  * ECM: endoscopic camera manipulator, active arm holding the stereo
    endoscope (Si and some S, see :ref:`generation support
    <davinci-generations>`)

The following components are not supported:

* Surgeon's console for Si
* Surgeon's console buttons on arm rest
* Focus controller on Si
* Original video pipeline with icons, etc.
* Audio between the surgeon's console and the video tower
