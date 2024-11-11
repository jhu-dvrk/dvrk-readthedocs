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
deployed in more than :ref:`40 different institutions <groups>`
worldwide.

The da Vinci
============

The first da Vinci system was developed in the late 1990s and approved
by the FDA for clinical use at the turn of the century.  The da Vinci
is designed for RAMIS (robotically assisted minimally invasive
surgery).  The main advantages of a teleoperated (robotically
assisted) laparoscopic surgery over a manual minimally invasive
surgery are:

* Extra dexterity at the end of the instrument (Intuitive's `EndoWrist
  <https://www.davincisurgerycommunity.com/instrumentation>`_).
* Depth perception. Instead of looking at a wall mounted monitor, the
  operator is seated at a console which can provide a stereo display.
* Motion scaling. Since the patient side manipulators are
  mechanically decoupled from the operator's hand, it is possible to
  scale down the motion. This allows operators to perform finer
  tasks.
* Hand-eye registration. The system can determine the pose and
  orientation of the patient side manipulators with respect to the
  endoscopic camera. Using this information, it is possible to compute
  the change of orientation (in 3D) to be applied to the surgeon's
  moves so the directions remain natural. If the surgeon moves
  to the left, the instrument will move to the left in the stereo
  display regardless of the camera pose.
* The operator is seated at an ergonomic console during most of the
  surgery instead of standing above the patient holding instruments
  and looking at a monitor.

The kit
=======

The dVRK's main goal is to re-use as much hardware as possible from
retired clinical da Vinci systems.  Designing and manufacturing robots
(both on the patient and operator sides) is a costly endeavor that
most research institutions cannot support.  Therefore, giving a second
life to these systems makes a lot of sense.

Besides the mechanical arms, the da Vinci also has its own
electronics and software.  These parts are much harder to share with
the community due to obsolescence and intellectual property issues.
Fortunately, the mechatronics and software are also a bit easier to
develop and distribute at a reasonable cost for research
institutions. To summarize, the :ref:`kit <kit>` is composed of:

1. Manipulators and other hardware (endoscope, stereo display...) from
   retired da Vinci systems
2. Controllers, software and some support from the dVRK group at the
   Johns Hopkins University (all open source)
3. Computers are not provided with the dVRK controllers.  PC and
   hardware setup, configuration and calibration are performed by the
   dVRK owner

Since each dVRK group might receive different hardware, :ref:`each
"kit" is different <dvrk-examples>`.

Credit/Citation
===============

See the :ref:`publication section <credit>`.
