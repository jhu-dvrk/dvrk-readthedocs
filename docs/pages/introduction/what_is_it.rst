*****************
What is the dVRK?
*****************

The da Vinci Research Kit (dVRK) is an “open-source mechatronics”
system, consisting of electronics, firmware, and software that is
being used to control research systems based on retired da Vinci
systems from `Intuitive Surgical Inc
<https://www.intuitive.com>`_. You can find a more detailed
description of the `dVRK on the Intuitive Foundation's site
<https://www.intuitive-foundation.org/dvrk/>`_. The dVRK is now
deployed in close to :ref:`40 different institutions <groups>`
worldwide.

The da Vinci
============

The first da Vinci systems were developed in the late 20th century
and approved by the FDA for clinical use at the turn of the century.
The da Vinci is designed for RAMIS (robotically assisted minimally
invasive surgery).  The main advantage of a teleoperated laparoscopic
surgery over a classic minimally invasive surgery are:

* Extra dexterity at the end of the instrument (patented by Intuitive
  as Endo-Wrist).
* Depth perception. Instead of looking at a wall mounted monitor, the
  operator is seated at a console which can provide a stereo display.
* Motion scaling. Since the patient's side manipulators are
  mechanically decoupled from the motion of the operator's hand, it is
  possible to scale down the motion.  This allows slower and more
  precise motions.
* Hand-eye registration. The system can determine the pose and
  orientation of the patient's side manipulators with respect to the
  endoscopic camera.  This transformation can then be applied to the
  motion of the operator with respect to their view to maintain a
  natural (intuitive) orientation.
* The operator is seated during most of the surgery instead of
  standing above the patient.

The kit
=======

The dVRK's main goal is to re-use as much hardware as possible from
retired clinical da Vinci systems.  Designing and manufacturing robots
(both on the patient and operator sides) is a costly endeavor that
most research institutions can't support.  Therefore giving a second
life to these systems makes a lot of sense.

Besides the mechanical arms, the da Vinci also has it's own
electronics and software.  These parts are much harder to share with
the community due to obsolescence and intellectual property issues.
Fortunately, the mechatronics and software are also a bit easier to
develop and distribute at a reasonable cost for research
institutions. To summarize, the :ref:`kit <kit>` comes from:

1. Manipulator and other hardware from retired da Vinci systems
2. Controllers, software and some support from the dVRK group at the
   Johns Hopkins University (all open source)
3. Computers, setup, configuration and calibration performed by the
   dVRK owner

Since each dVRK group might receive different, :ref:`each setup is
different <dvrk-examples>`.
