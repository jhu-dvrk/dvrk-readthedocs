.. _davinci-classic:

Introduction
############

History
=======

The first generation da Vinci, IS1200, also known as Standard or
Classic, was developed in the late 1990s.  It was retired from
clinical use in 2012.

.. note::

   Intuitive uses the name "Standard" for the first generation by
   contrast to the second generation da Vinci which came with a high
   definition :ref:`video pipeline <video-sources>` (standard
   definition vs. high definition).  We found "Standard" a bit
   confusing, so we tend to call the first generation system "Classic".

.. note::

   The da Vinci S (second generation) used a surgeon's console very
   similar to the Classic systems.  For the dVRK community, MTMs from
   a da Vinci S are still called MTM Classic.  The same applies to the
   head sensor, foot pedals...

Main components
===============

.. figure:: /images/general/ISI-da-Vinci-Classic-overview-labeled.png
   :width: 600
   :align: center

   da Vinci Classic: surgeon's console, video tower and patient's cart

.. figure:: /images/general/ISI-da-Vinci-S-overview-labeled.png
   :width: 600
   :align: center

   da Vinci S: surgeon's console, patient's cart and video tower


dVRK integration
=================

The following components can be used for the dVRK (see :ref:`dVRK
setup <setup-classic>`)

* Surgeon's console (Classic and S)

  * MTMs: master tele manipulators, the two arms the operator uses to
    control the patient's side, 6 dof haptic devices with gripper
  * Foot pedals: pedals used to switch the control modes while
    operating, select which PSM or ECM, clutch, focus...
  * HRSV: high resolution stereo viewer, aluminum frame with optics,
    mirrors and CRT monitors for both eyes
  * Head sensor: sensor used to detect if the operator is looking at
    the stereo display and enable the teleoperation
  * Console lift motor: adjustable height for the HRSV for better ergonomic
  * Console frame: can be reused to hold everything as opposed to
    building a custom frame

* Video tower (Classic and S), see :ref:`dVRK video support <video>`

  * Light source
  * Camera control units (SD and HD)
  * Focus controller
  * The rack itself if provided along the dVRK

* Patient's cart (Classic only)

  * SUJ: setup joints, cart with central column and up to 4 passive
    arms with brakes to position the active arms around the patient
  * PSMs: patient side manipulators, active arms holding the
    laparoscopic instruments with EndoWrist
  * ECM: endoscopic camera manipulator, active arm holding the stereo
    endoscope

The following components are not supported:

* Surgeon's console buttons on arm rest
* Original video pipeline with icons, etc.
* Audio between the surgeon's console and the video tower
