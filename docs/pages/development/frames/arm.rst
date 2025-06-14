.. _frames_arm:

Single arm
##########


For each individual arm, the cartesian positions are based on:

* DH parameters: defined in JSON files, used to compute the forward
  kinematics using joint positions
* Tool tip offset (optional):

  * Defined in the kinematic JSON file, ``tooltip-offset`` is a
    constant offset appended at the end of the kinematic chain
    (PSM and ECM only, not MTM).  The tooltip offset for the ECM is
    determined by the type of endoscope (straight, up, down) and
    automatically managed by the ECM arm class.  The PSM tooltip
    offset is determined by the type of instrument (found in
    ``share/tool``).
  * Most users should never deal with these, they are predefined for
    given PSM tools and ECM endoscopes

* Base Frame (``base_frame`` in your system JSON file):

  * Prepended to the whole kinematic chain
  * Can be:

    * defined in the system JSON configuration per arm using ``base_frame``
    * set at runtime by command or ROS topic ``set_base_frame``

For example, the MTM kinematic (DH parameters) start from the link 0,
i.e. close to the attachment point/shoulder (see
``share/kinematics/mtmr.json`` in
https://github.com/jhu-dVRK/sawIntuitiveResearchKit) and has Z
pointing up, X to the left and Y towards the user.  But the ISI
convention expects that X points to the left when viewed from the
stereo display, Y should point up and Z away from the user.
Furthermore, the ISI convention places the origin in the middle of the
stereo display (i.e. between the operator's eyes).  In practice MTMs
are always mounted rigidly to a frame, so we need to apply a constant
rotation and translation to match the ISI convention.  The two masters
are also mounted apart from each other, so there is also a positive X
translation for MTML and a negative X translation for MTMR.  The
change of reference frame should be defined using the ``base_frame``
in the system configuration file (see
``jhu-dVRK/system-MTMR-PSM1-MTML-PSM2-Teleop.json`` in
https://github.com/dvrk-config/dvrk_config_jhu).

For a change of reference frame at runtime one should use the
``set_base_frame`` ROS topic.  For example, this is used to make sure
the PSMs are always defined with respect to the endoscope tooltip
(camera frame) if the camera can move (i.e. mounted on an ECM or other
robot).

To summarize:

* When used with the setup joints, the system class propagates the
  different base frames to make sure the PSMs are defined with respect
  to the camera frame (see :ref:`SUJ <frames_SUJ>`).
* If you have an ECM **and** don't have access to the setup joints
  **but** have a way to register the PSMs to the camera/ECM, you
  should use the SUJ Fixed class.  We offer a simple method to
  register the PSMs to the ECM:
  https://github.com/jhu-dvrk/dvrk_camera_registration
* Finally, if the camera is fixed, you can use ``base_frame`` in your
  system configuration (similar to MTMs, see
  ``jhu-dVRK/system-MTMR-PSM1-MTML-PSM2-Teleop.json`` in
  https://github.com/dvrk-config/dvrk_config_jhu).  Note that the
  ``base_frame`` depends on where your PSMs are mounted with respect
  to your fixed camera.

There are two possible cartesian positions to query:

* ``measured_cp``: ``base_frame * DH * tooltip-offset``
* ``local/measured_cp``: ``DH * tooltip-offset``

Surgeon's side
**************

The base frames defined in the ``system.json`` are such that the MTM
tips (grippers) are defined with respect to the stereo display.  By
default, we assume the stereo display is pointing down with a 60-degree
angle from the horizontal.  This is the orientation of the HRSV
on the Standard (aka Classic) and S da Vinci systems.  If your display
is mounted at a different angle, adjust your ``system.json``
accordingly.

.. figure:: /images/transformations/dVRK-transformations-surgeon-console.*
   :width: 600
   :align: center

   Surgeon's side transformations

Patient's side
**************

The base frames defined in your ``system.json`` depend on how your
PSMs are mounted.  If you have a fixed camera, you can use or hand-eye
calibration method:
https://github.com/jhu-dvrk/dvrk_camera_registration.  If you have an
ECM, see :ref:`SUJ <frames_SUJ>`.

.. figure:: /images/transformations/dVRK-transformations-PSMs-kit.*
   :width: 600
   :align: center

   Patient's side without any type of SUJ
