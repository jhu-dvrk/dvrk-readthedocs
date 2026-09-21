.. include:: /includes/logic-view-soft-bridge.rst

.. _devel-bridge-ros:

.. _bridge-ros:

.. _devel-ros-clients:

ROS
###

Description
***********

The simplest way to write an application for the dVRK is to use ROS.
The ROS bridge is provided by `cisst-ros <https://github.com/jhu-cisst/cisst-ros>`_,
a generic |cisstMultiTask|_ bridge that auto-discovers all provided and required
interfaces of any cisst/SAW component and exposes them as ROS topics and
services. It supports both ROS 1 and ROS 2 from a single code base.

.. figure:: /images/software/dVRK-component-ROS-teleop.*
   :width: 400
   :align: center

   Using multiple processes with ROS

The built-in executable ``dvrk_system`` (package ``dvrk_robot``) links the ROS
bridge statically — no extra ``-m`` flag is required:

.. code-block:: bash

   # ROS 2
   ros2 run dvrk_robot dvrk_system -j your_system_config.json

   # ROS 1
   rosrun dvrk_robot dvrk_system -j your_system_config.json

The dVRK can also run **without ROS** on any operating system using the
``sawIntuitiveResearchKitSystem`` executable, which links only the Qt interface.
This is useful for embedded deployments or platforms where ROS is unavailable.


Python client
*************

.. _CRTK-Python-client:

The ``dvrk_python`` package provides a Python API that communicates with the
dVRK system over ROS. It is based on the
`CRTK Python client libraries <https://github.com/collaborative-robotics/crtk_python_client>`_
and follows the `CRTK naming convention <https://crtk-robotics.readthedocs.io>`_.

If you retrieved the dVRK software using the :ref:`compilation instructions
<compilation>`, ``dvrk_python`` will already be in your workspace. It is also
used for most calibration scripts.

Key features:

* A ROS version-independent API (``crtk.ral`` — ROS Abstraction Layer) so the
  same script works with ROS 1 **or** ROS 2.
* Wrappers around ROS publishers/subscribers — no boilerplate needed.
* Convenient classes for arms, the system, foot pedals... with methods covering
  most features.
* Cartesian data returned as ``PyKDL.Frame``; vectors and matrices as
  ``numpy`` arrays.
* All state-retrieval methods return a ``(data, timestamp)`` tuple. The
  timestamp is ``0`` when data is invalid (e.g. no instrument on a PSM):

  .. code-block:: python

     p, ts = robot.measured_jp()
     if ts != 0.0:
         do_something_useful()
     else:
         handle_invalid_data()

Usage
=====

Start the dVRK system, then in Python:

.. code-block:: python

   import crtk, dvrk

   ral = crtk.ral('dvrk_python_node')   # ROS Abstraction Layer
   p = dvrk.psm(ral, 'PSM1')            # name must match ROS namespace
   ral.check_connections()               # wait for topics (default 5 s timeout)
   ral.spin()                            # required for ROS 2; no-op on ROS 1

   p.enable()
   p.home()

   # Joint state
   p.measured_jp()   # positions (numpy.array), timestamp
   p.measured_jv()   # velocities
   p.setpoint_jp()   # PID setpoint

   # Cartesian state (PyKDL.Frame, timestamp)
   p.measured_cp()
   p.setpoint_cp()

   # Move in joint space
   import numpy
   p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

   # Move in Cartesian space
   import PyKDL
   goal, _ = p.setpoint_cp()
   goal.p[2] += 0.05            # 5 cm in z
   p.move_cp(goal).wait()

   import math
   goal, _ = p.setpoint_cp()
   goal.M.DoRotX(math.pi * 0.25)
   p.move_cp(goal).wait()

MTM wrench example (hold the arm before running):

.. code-block:: python

   from dvrk import mtm
   import crtk, numpy

   ral = crtk.ral('mtm_node')
   m = mtm(ral, 'MTML')
   ral.check_connections()
   ral.spin()

   m.set_wrench_body_orientation_absolute(True)
   m.body.servo_cf(numpy.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))  # 2 N in y
   m.lock_orientation_as_is()
   m.set_gravity_compensation(True)

Performance
===========

The ``dvrk.arm``, ``dvrk.psm``, ``dvrk.mtm``... classes subscribe to *all*
available topics for convenience, which adds overhead. For production code,
build your own class and add only the ``crtk.utils`` features you need. See
``scripts/dvrk-bag-replay.py`` in ``dvrk_python`` for an example.

.. warning::

   The dVRK system publishes state at 100 Hz by default. Adjust with the
   ``-p`` command line option of ``dvrk_system`` if you need a different rate.

Useful links:

* `dvrk_python <https://github.com/jhu-dvrk/dvrk_python>`_ — Python client and scripts
* `CRTK Python client <https://github.com/collaborative-robotics/crtk_python_client>`_
* `CRTK documentation <https://crtk-robotics.readthedocs.io>`_


Matlab client
*************

.. note::

   The Matlab CRTK client library is not actively supported.

* `dvrk_matlab <https://github.com/jhu-dvrk/dvrk_matlab>`_ (ROS 1)
* `ros2_dvrk_matlab <https://github.com/jhu-dvrk/ros2_dvrk_matlab>`_ (ROS 2)
* `crtk_matlab_client <https://github.com/collaborative-robotics/crtk_matlab_client>`_ (ROS 1)
* `ros2_crtk_matlab_client <https://github.com/collaborative-robotics/ros2_crtk_matlab_client>`_ (ROS 2)
* `dvrk-gravity-compensation <https://github.com/jhu-dvrk/dvrk-gravity-compensation>`_ — example


arm_from_ros component
**********************

The ``dvrk_arm_from_ros`` component is a predefined cisst/SAW component that
makes the dVRK system treat any CRTK-compatible ROS interface as a local arm.
When used, the system communicates with the external arm over ROS topics instead
of directly driving hardware.

.. figure:: /images/software/dVRK-component-PSM-from-ROS.*
   :width: 400
   :align: center

   Using a ROS bridge component

Typical use cases:

* Simulation environments (AMBF, IsaacSim...) that expose virtual arms via ROS
* Running teleoperation across a network — the remote PSM exposes CRTK topics
  and the local MTM process treats it as a local arm

How it works (example: ``servo_cp`` from teleoperation to a remote PSM):

1. The teleoperation component calls ``servo_cp`` on its required PSM interface.
2. ``dvrk_arm_from_ros`` receives the command over its provided PSM-like interface,
   converts it to a ROS message and publishes it to ``/PSM1/servo_cp``.
3. ROS delivers the message to the PSM process.
4. The cisst-ros bridge in the PSM process subscribes to ``/PSM1/servo_cp``,
   converts the message back to a cisst type and calls the real PSM component.

.. note::

   Topic name mismatches are silent — use ``ros2 topic list`` and
   ``ros2 topic info`` to diagnose connection issues.

Example system configuration:
`jhu-dVRK/system-MTML-PSM1_ROS-Teleop.json
<https://github.com/dvrk-config/dvrk_config_jhu>`_.


Discussion
**********

Pros
====

* No need to understand internal dVRK components or cisst libraries.
* Any ROS-supported language: C++, Python, Java...
* Your application runs in a separate process (or separate computer),
  keeping computing load off the dVRK system.
* Full ROS tooling: ``rosbag``, RViz, PlotJuggler...

Cons
====

* Serialization/deserialization cost — generally acceptable for modern hardware.
  Control loops at 500 Hz or more are achievable in C++ or Python; Matlab may
  not sustain frequencies that high.
* Latency is typically under 1 ms for most messages. Messages include a
  ``timestamp`` header so you can track when data was generated.

Notes
=====

* The :ref:`dVRK system <system>` publishes events (e.g. ``operating_state``)
  immediately and state data (e.g. ``measured_js``) at a configurable
  periodic rate (default 100 Hz, adjust with ``-p``). The arm components run at
  1.5 kHz so publishing faster than 1.5 kHz is not useful.
* `cisst-ros <https://github.com/jhu-cisst/cisst-ros>`_ uses multiple threads
  to minimize latency for synchronous messages.
* ROS topics exposed by the dVRK can be configured: the system node can expose
  IO and PID topics in read-only or read-write mode
  (see :ref:`system application <system>`).
