.. include:: /includes/logic-view-console.rst

.. _components-custom:

*****************
Custom components
*****************

.. _components-derived:

Derived components
==================

Since the dVRK stack relies on |cisstMultiTask|_ components, you can replace any
existing component with your own as long as the interface is compatible. If your
changes are modest, the easiest approach is to start from the existing class and
derive from it. Advantages of derived classes:

* All the provided and required interfaces the system expects are already defined
  (ROS bridge, Qt widget, PID, IO...) — no boilerplate needed.
* Existing configuration parameters are inherited — you can reuse them.
* Component creation code is already in the base class — less code to maintain.

As of 2023 the dVRK system supports derived classes for the arm and the PSM
teleoperation. Documented examples:

* **Derived** ``mtsTeleOperationPSM`` — ``examples/derivedTeleOperationPSM``
  in `sawIntuitiveResearchKit <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_.
  Shows a single derived C++ class.

* **Derived** ``mtsIntuitiveResearchKitPSM`` — ``examples/derivedPSMQtROS``
  in `sawIntuitiveResearchKit <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_.
  Shows a derived C++ class together with a custom Qt widget and a custom ROS
  bridge so you can communicate with the derived class using custom ROS messages.


Writing your own
================

If no existing dVRK component is close to what you need, you can write a
|cisstMultiTask|_ component from scratch and connect it to the running system
using the component manager.

Starting points:

* `cisstMultiTask tutorial <https://cisst.readthedocs.io/en/main/libraries/cisstMultiTask/tutorial.html>`_
  — concepts, provided/required interfaces, state tables, events
* `cisstMultiTask concepts <https://cisst.readthedocs.io/en/main/libraries/cisstMultiTask/concepts.html>`_
  — threading model, ExecIn/ExecOut, queued commands
* ``examples/`` directory in
  `sawIntuitiveResearchKit <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_
  — working examples including Qt widgets and custom ROS bridges

A custom component can be:

* **In-process** — added via a component manager configuration file (``-m``
  option), sharing the same process and thread model as the dVRK components.
* **Out-of-process** — a separate process communicating over one of the
  :ref:`middleware bridges <devel-bridges>`.

.. note::

   All dVRK APIs (C++, Python over ROS, JSON over UDP, OpenIGTLink) follow the
   :ref:`CRTK naming convention <API-introduction>`. Custom components should
   do the same to stay interoperable.
