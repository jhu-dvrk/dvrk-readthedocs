.. _widgets:

Widgets
#######

General information
*******************

There is some built-in documentation for some dVRK widgets, you
can see it when you move the mouse over said widget.  There are also a
few convenient keyboard shortcuts (for example ``ctrl+q`` to quit).
Moving the mouse over a button will display the corresponding
shortcut.

To increase readability, the dVRK widgets display positions in
millimeters and degrees even though all internal values use SI units
(meters and radians).

Most dVRK related widgets have a *Direct control* toggle button.  By
default, *Direct control* is disabled and, for most users, it should
remain disabled.

.. warning::

   *Direct control* should only be used for debugging and calibration
   only.  When enabled, the user has a more direct access to the
   controller's state and this can lead to unstable conditions... and
   potential damage to the dVRK arms.

Generic widgets
***************

The dVRK components use a few widgets included in the *cisst* libraries.

Timing
======

todo

Messages
========

todo

t to toggle, c to clear

3D pose
=======

  On top, from left to right, the widget display the names of the
  moving and reference frames (e.g. PSM1/ECM).  If the cartesian pose
  is meaningless (e.g. arm not homed, no instrument on PSM), the
  widget will show *Invalid* in red.  The time displayed along the
  status is the time the pose was computed since the program started.

  .. figure:: /images/gui/gui-pose-3D.png
     :width: 200
     :align: center

     Pose widget (3D view)

  The rotation is displayed in the middle.  The default is a 3D view,
  but one can change to a matrix, quaternion or axis/angle using the
  right-click menu.

  The position vector is displayed at the bottom, in millimeters.

  .. figure:: /images/gui/gui-pose-axis-angle.png
     :width: 200
     :align: center

     Pose widget (axis/angle view)

Wrench
======

todo

  .. figure:: /images/gui/gui-wrench-3D.png
     :width: 200
     :align: center

     Wrench (3D view)

  .. figure:: /images/gui/gui-wrench-2D.png
     :align: center

     Wrench (time plot)

2D plot
=======

todo

Customization
*************

Starting with the dVRK 2.0, we added support for a "pseudo" dark mode
and Qt styles.  If you're using a Qt based window manager you will
likely not use these features (e.g. KDE).  For the default Ubuntu
window managers, these extra options allow some user customization of
the dVRK GUIs.

To activate the dark mode, add the option ``-D`` when starting the
dVRK console application.  This applies to the plain application
``sawIntuitiveResearchKitQtConsoleJSON`` as well as the ROS node
``dvrk_robot/dvrk_console_json``.

.. figure:: /images/gui/dvrk-style-dark.png
   :align: center

   "dark" style

To change the Qt style, use the option ``-S``.  To figure out which Qt
styles are available, use a dummy style that doesn't exist: ``-S
unicorn`` (let's hope no one will ever create a Qt style named
"unicorn").  The application will fail to launch, but it will display a
list of available styles.

To install some extra styles:

* Ubuntu 18.04: ``sudo apt install qt5-style-plugins kde-style-oxygen-qt5 kde-style-qtcurve-qt5``
* Ubuntu 20.04: ``sudo apt install qt5-style*``

Since we use Qt for all GUIs these options should work on all OSs, but
we've only tested them on Linux.

.. figure:: /images/gui/dvrk-style-oxygen.png
   :align: center

   Oxygen style on Ubuntu 18.04

.. figure:: /images/gui/dvrk-style-qt-curve.png
   :align: center

   QtCurve style on Ubuntu 18.04

.. figure:: /images/gui/dvrk-style-ukui-dark.png
   :align: center

   ukui-dark style on Ubuntu 20.04
