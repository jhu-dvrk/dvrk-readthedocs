Introduction
############

The original application for the dVRK/da Vinci is RAMIS (Robotically
Assisted Minimally Invasive Surgery).  For this application, it makes
sense to use the same convention for the system coordinates on both
the surgeon's and patient's sides.  The ISI (Intuitive Surgical Inc)
convention expects that X points to the left when viewed from the
stereo display, Y should point up and Z away from the user.  Using the
same convention greatly simplifies the implementation of teleoperation
tasks.  From there it is important to always define the arm motions
with respect to the task:

* MTMs with respect to the stereo display
* PSMs with respect to the camera coordinate systems

To make sure coordinate systems are properly aligned, one needs to
first find the transformation between the arm base frame and the task
frame (display or camera) using either a kinematic chain (SUJ) or a
registration method.  Once this transformation is found, it has to be
used as *base frame* for the arm.

