.. _teleop-ecm:

.. include:: /includes/logic-view-soft-teleop.rst

ECM teleoperation
#################

Introduction
************

For general background on dVRK teleoperation components and how to customize or
derive from them, see the :ref:`PSM teleoperation introduction <teleop-psm>` and
the :ref:`application development section <devel-options>`.

The dVRK provides two ECM teleoperation components derived from the common base
class ``mtsTeleOperationECMBase``:

* ``mtsTeleOperationECMTwoHands`` (``TELEOP_ECM_TWO_HANDS``, default): uses
  both ``MTML`` and ``MTMR`` to hold the "world" with two hands and control the
  ECM, mimicking the clinical Intuitive Surgical Inc (ISI) systems.
* ``mtsTeleOperationECMOneHand`` (``TELEOP_ECM_ONE_HAND``): uses a single
  ``MTM`` (position and wrist orientation) to hold the "world" and drive the
  ECM when only one MTM is available or configured.

Behavior
********

Two-hands behavior
==================

For the PSM, the teleoperation logic maps the motion of the instrument to a
single operator input (MTM).  For the ECM, one could imagine that one hand
(MTM) would be enough to control the camera motion.  On a full console, directly
mapping a single MTM to the ECM presents several issues:

* The ECM has 4 DOFs (3 translations and 1 rotation) while the MTM has 6 DOFs
  (3 translations and 3 rotations).  This means that some of the MTM motions
  are not feasible for the ECM.

* MTM wrist rotations would be used to control the first two joints of the ECM.
  These joints are not as light as the wrist of the PSM instruments.  Small
  wrist motions on the operator's side would lead to aggressive motions on the
  ECM.

* Once the ECM teleoperation ends, the MTM orientation would not match the
  PSM's end-effector orientation.  This would require re-aligning the MTM more
  often.

Instead, the default ECM teleoperation is performed using both MTMs.  The
operator feels like they are holding the world with both hands and moving the
world instead of the camera itself.

On the patient's side, the ECM motion is constrained by the RCM.  The camera can
tilt up and down as well as pan left and right.  It can also be moved in and out
and roll along its axis.  The ECM is also pretty heavy and can't move as fast as
the PSMs.  To mimic these constraints on the operator's side, haptic feedback is
used (when ``MTM_is_haptic`` is ``true``) to:

* Maintain the two MTMs at a constant distance from each other to mimic the user
  grabbing the world.

* Keep the two MTMs in a plane perpendicular to the vector going from the
  midpoint of the two MTMs to the RCM.  This ensures that the MTMs move in a
  way that is consistent with the ECM motion.

* Slow down the overall ECM motion by applying a strong friction haptic feedback
  on both MTMs.

.. note::

   The MTMs have to report their Cartesian positions with respect to the stereo
   display; the origin is roughly between the operator's eyes.

Then the motions allowed are the following:

* Moving both MTMs **up** or *down* will tilt the camera **down** or *up*
  respectively.

* Moving both MTMs **left** and *right* will pan the camera **right** and *left*
  respectively.

* Moving both MTMs **closer** or further *away* from the operator will move the
  camera **in** and *out*.

* Turning the two MTMs like a handlebar will make the camera rotate along its
  axis.  When the operator turns **clockwise**, the camera turns
  **counter-clockwise**.

.. note::

   Since the operator can't move the world itself, the camera has to move using
   the inverse transform to give the impression the world moves.

This approach only relies on the position (x, y, and z) of both MTMs, not their
wrist orientation.  During two-hands ECM teleoperation, the MTM wrist
orientations are continuously updated to match the PSMs' orientation with
respect to the camera while the ECM is moving.  This way, when the ECM
teleoperation stops, the MTMs are already aligned and the PSM teleoperation can
start immediately.

One-hand behavior
=================

When only a single MTM is configured (using the ``MTM`` configuration field or
``TELEOP_ECM_ONE_HAND``), ``mtsTeleOperationECMOneHand`` lets the operator hold
the "world" with one hand:

* Translating the MTM moves the virtual world up/down, left/right, and in/out
  (driving the first three ECM joints).
* Rotating the MTM wrist rotates the virtual world to control the camera roll
  (driving the fourth ECM joint).
* The MTM is placed in gravity compensation with zero Cartesian wrench; no
  two-hand haptic constraint forces or wrist orientation locks are applied.

States
******

``mtsTeleOperationECMBase`` maintains an internal state machine corresponding
to the stages of ECM teleoperation:

``DISABLED``
  Nothing to do, the teleoperation doesn't need to run.

``SETTING_ARMS_STATE``
  In this state, the teleoperation component attempts to enable and home the
  required arms (``state_command("enable")`` and ``state_command("home")``) and
  monitors their operating states (``operating_state``).  Once the ECM and
  MTM(s) are ready (or immediately for non-haptic MTMs in two-hands mode), the
  component moves to ``CHECKING_DATA``.

``CHECKING_DATA``
  Verifies that the Cartesian and joint states read from the arms
  (``ECM/measured_cp``, ``ECM/setpoint_js``, and MTM ``measured_cp``) are
  valid.  In two-hands mode with ``check_MTMs_workspace`` enabled, it also
  checks that both ``MTML`` and ``MTMR`` positions are within the expected
  surgeon-console workspace so that a misconfigured ``base-frame`` is caught
  before enabling motion.  Once valid, the component transitions to
  ``ENABLED``.

``ENABLED``
  Follow mode is active (unless clutched).  When entering ``ENABLED``, the
  initial MTM pose(s) and initial ECM joint positions (``ECM/setpoint_js``) are
  saved, and at each cycle the component sends joint position goals to the ECM
  (``ECM/servo_jp``).

Clutch
  * When the operator presses the console clutch to reposition their hands,
    following stops, zero wrench is applied to the MTM(s) with gravity
    compensation enabled, and in two-hands haptic mode the MTM wrist
    orientations remain locked.
  * When the operator releases the clutch, the component transitions back
    through ``SETTING_ARMS_STATE`` and ``CHECKING_DATA`` to ``ENABLED``,
    recording a new initial snapshot of the MTM(s) and ECM so motion resumes
    smoothly from the new hand positions.

Implementation
**************

As for the PSM teleoperation, the ECM teleoperation records the initial pose of
the MTM(s) and the initial joint positions of the ECM (``ECM/setpoint_js``) when
follow mode starts.  All subsequent ECM joint positions are computed from the
current MTM pose(s) compared to the initial ones and sent in joint space via
``ECM/servo_jp``.

For **two-hands** teleoperation (``mtsTeleOperationECMTwoHands``):

* Find the distance between the MTMs and the reference frame, i.e. the stereo
  display.

* Use the average depth (midpoint between the two MTMs) to create a virtual
  image plane, aka the world.

* Project the MTM positions on that plane to define two virtual handles on the
  world, and apply haptic spring and viscous friction forces
  (``body/servo_cf``) to keep the two hands at their initial separation on the
  plane.

* Track the position of the midpoint between the two handles to compute the
  motion of ECM joints 1, 2, and 3 (pan, tilt, and scaled in/out insertion).

* Track the orientation of the vector between the two handles to compute the
  motion of ECM joint 4 (roll), rotating the pan and tilt joint increments by
  the total endoscope roll angle (including any ``camera_roll_offset``).

* Continuously update ``lock_orientation`` on both MTMs with the inverse of the
  ECM rotation change so the MTM wrists remain aligned with the PSMs in the
  camera frame.

For **one-hand** teleoperation (``mtsTeleOperationECMOneHand``):

* Create a virtual image plane ("world") at a nominal depth (0.35 m) in front
  of the stereo display.

* Apply the MTM's scaled translation to move the center of the world and the
  MTM's relative wrist rotation to rotate the world, then use the same planar
  projections and camera-roll compensation to compute the 4 ECM joint goals.

Configuration options
*********************

The following parameters can be configured under ``configure_parameter`` in the
console JSON configuration file (see ``dvrk-teleop-ECM.schema.json``):

* ``scale``: scaling factor applied to insertion motion (two-hands) or MTM
  translation (one-hand).
* ``camera_roll_offset``: angular offset (in radians) between ECM joint 4 and
  the endoscope optical roll axis.
* ``MTM_is_haptic``: set to ``false`` for non-motorized or alternate MTMs (e.g.
  Haply, OpenXR controllers) to disable MTM forces, gravity compensation,
  orientation locking, and workspace checks.
* ``check_MTMs_workspace``: check MTM positions against the standard surgeon
  console workspace before enabling two-hands teleoperation (default ``true``,
  automatically disabled when ``MTM_is_haptic`` is ``false``).
* ``haptic_feedback_ratio``: scale factor in ``[0.0, 1.0]`` applied to the
  haptic wrench sent to the MTMs in two-hands mode (default ``1.0``).

Main limitations
****************

* ECM mechanical joint limits are not checked by the teleoperation logic.
* The current MTM inverse kinematics relies on a simple iterative algorithm and
  is pretty bad at handling the MTM 7 DOFs.  When entering or running two-hands
  ECM teleoperation, the MTM wrists might move in odd ways.

Code
****

* dVRK constants:
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/include/sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h
  (some related to ECM teleoperation)
* Base class:
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/include/sawIntuitiveResearchKit/mtsTeleOperationECMBase.h
  and
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/code/mtsTeleOperationECMBase.cpp
* Two-hands teleoperation:
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/include/sawIntuitiveResearchKit/mtsTeleOperationECMTwoHands.h
  and
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/code/mtsTeleOperationECMTwoHands.cpp
* One-hand teleoperation:
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/include/sawIntuitiveResearchKit/mtsTeleOperationECMOneHand.h
  and
  https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/main/core/components/code/mtsTeleOperationECMOneHand.cpp

API
***

See :ref:`teleoperation API <api-teleoperation>`.
