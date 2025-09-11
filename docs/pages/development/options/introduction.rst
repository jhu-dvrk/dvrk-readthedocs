.. _devel-options-intro:

.. include:: /includes/logic-view-user-app.rst

Introduction
############

The dVRK software stack is meant to be as open as possible, so it can
be integrated in an end-user application.  Once you've configured your
subset of dVRK arms to use in your ``system.json``, the dVRK system
class will automatically set up some Qt widgets for visual debugging
as well as some ROS topics, services and tf2 transforms.  This is
described in the :ref:`software architecture <architecture>` page.
The API is described in the :ref:`dVRK API 2.x <devel-api>`.  The
dVRK API uses the CRTK naming convention as much as possible, so you
should familiarize yourself with `CRTK
<https://crtk-robotics.readthedocs.io>`_.

.. figure:: /images/software/dVRK-component-standard.*
   :width: 400
   :align: center

   Default components for the dVRK process

If you decide to use ROS and treat the dVRK system as a "black box", you will
end-up with two (or more processes).  ROS is one example of middleware between
the processes, we also provide some support for OpenIGTLink and plain UDP
sockets.  You can write your ROS nodes using ROS directly or use the :ref:`CRTK
Python Client < CRTK-Python-client>`.  The latter offers a simple Python API
that doesn't require any ROS knowledge.

Alternatively, you can opt to implement your application's logic using a new
*cisstMultiTask* component.  In this case, your application will be a single
process with multiple threads.  If C++ is your programming language of
predilection, you can write your :ref:`own (custom) components
<components-generic>` or :ref:`derive an existing one <components-derived>`.  If
you prefer Python, you should try the IRE (:ref:`embedded Python interpreter
<components-IRE>`).

.. note::

   Even though the exact syntax depends on the programming language, all the
   dVRK APIs, C++, Python, ROS topics, JSON over UDP (|sawSocketStreamer|) use
   the :ref:`CRTK naming convention! <API-introduction>`

.. note::

   There is at least two ways to write dVRK applications using Python. Over ROS
   topics with the CRTK Python Client or with the embedded Python interpreter
   (IRE). Pick based on your requirements.

