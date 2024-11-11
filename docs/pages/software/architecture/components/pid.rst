.. _pid:

PID
###

Functionalities
***************

The class ``mtsPID`` is part of the `sawControllers
<https://github.com/jhu-saw/sawControllers>`_ library.

It provides:

* A PID controller
* Support for disturbance/observer (in lieu of Integral)
* Clamp setpoint positions to stay within joint position limits
* Cap maximum PID output to stay under maximum joint torque allowed
* Simple PID tracking error, i.e. compares the joint setpoints and
  measured.  If the difference is higher than a given threshold, the
  PID component triggers an error.

Configuration files
*******************

Configuration files use JSON.  Files are usually shared across sites.
The default values for all arms (MTMs, PSMs and ECMs) can be found in
the main dVRK repository under ``share/pid``.

Applications
************

  * *sawIntuitiveResearchKitQtConsoleJSON* and ROS
    ``dvrk_robot/dvrk_console_json`` for regular use
  * *sawIntuitiveResearchKitQtPID* for debugging

Widgets
*******

By default the widget is used for display only.  One can change which
axis is used for the plotting section.

.. figure:: /images/gui/gui-Classic-MTM-pid.png
   :align: center

   PID widget for MTM

In *Direct control* mode, one can change the PID gains, safety
settings and PID setpoints.  This mode should be used only for PID
tuning using *sawIntuitiveResearchKitQtPID*.

.. figure:: /images/gui/gui-Classic-MTM-pid-direct.png
   :align: center

   PID widget for MTM in *Direct control* mode

.. note::

   For the dVRK, we use the :ref:`same thread<thread>` for the IO and
   PID components, so there is no timing widget in the PID widget.
   
