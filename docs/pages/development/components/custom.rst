.. include:: /includes/logic-view-console.rst

.. _components-custom:

Custom components
#################

.. _components-derived:

Derived components
******************

Since the dVRK stack relies on *cisstMultiTask* components (see
`tutorial
<https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts>`_),
one can technically replace any existing component by their own.  It's
possible but not necessarily easy nor the best approach.  If your
changes are modest, it might be easier to start from the existing
component and alter it.  The best way to do so is to derive from the
default dVRK class.  The main advantages of derived classes are:

* All the interfaces (provided and required) the system expects are
  already defined, so the system can connect the existing ROS bridge
  (1 or 2), Qt widget, PID, IO, etc.
* All the existing configuration parameters will still be there, so you
  can re-use them.
* The code related the component creation is already defined in the
  base class, so you'll have less code to manage.

As of 2023, the dVRK system supports derived classes for the arm and
the PSM teleoperation.  You can find documented examples for:

* Derived ``mtsTeleOperationPSM`` in
  ``examples/derivedTeleOperationPSM``
  (https://github.com/jhu-dvrk/sawIntuitiveResearchKit). This example
  hows a single derived C++ class.
* Derived ``mtsIntuitiveResearchKitPSM`` in
  ``examples/derivedPSMQtROS``
  (https://github.com/jhu-dvrk/sawIntuitiveResearchKit). This example
  shows a derived C++ class as well as a custom Qt Widget and ROS
  bridge (ROS 1) so one can communicate with the derived class with
  custom messages.


Writing your own
****************
