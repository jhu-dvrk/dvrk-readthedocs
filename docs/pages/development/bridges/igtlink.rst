.. include:: /includes/logic-view-soft-bridge.rst

.. _devel-bridge-igtlink:

.. _igtl:

sawOpenIGTLink
##############

Description
***********

|sawOpenIGTLink|_ is a dynamically loadable cisst/SAW bridge that uses the
`OpenIGTLink protocol <https://openigtlink.org>`_ instead of UDP/JSON. OpenIGTLink
is a well-established standard in the surgical robotics and medical imaging
community, with native support in `3D Slicer <https://www.slicer.org>`_.

The setup pattern is identical to |sawSocketStreamer|_: a component manager
file loaded with ``-m`` plus a bridge configuration file. The only difference
is that messages are transmitted using OpenIGTLink sockets and serialization,
so the client application must use an OpenIGTLink library (C++, Python, or any
other language with an OpenIGTLink binding).

Primary audience: researchers using 3D Slicer for visualization, image
guidance, or registration alongside the dVRK, on any platform (Linux, macOS,
Windows).

.. note::

   The dVRK developers have also worked on the `SlicerROS2
   <https://slicerros2.readthedocs.io>`_ module, which provides a more complete
   ROS 2 integration for 3D Slicer. If you are already running ROS 2,
   SlicerROS2 may offer a richer experience than the OpenIGTLink bridge.


How to
******

Installation
============

Clone the repository into your workspace:

.. code-block:: bash

   cd ~/ros2_ws/src/cisst-saw
   git clone https://github.com/jhu-saw/sawOpenIGTLink

Build alongside the dVRK:

.. code-block:: bash

   colcon build   # ROS 2
   # or: catkin build  (ROS 1)
   # or: cmake / make  (no ROS)

Configuration files
===================

You need at least two configuration files:

1. **Component manager file** (``-m`` option): tells |cisstMultiTask|_ to
   dynamically create and connect the ``sawOpenIGTLink`` component.
2. **IGTL configuration file**: lists which commands and events to bridge,
   using OpenIGTLink message types and socket settings.

Running
=======

.. code-block:: bash

   ros2 run dvrk_robot dvrk_system \
       -j your_system_config.json \
       -m your_igtl_manager.json

Examples
========

Configuration files for the dVRK are in the ``share/igtl`` directory of
`sawIntuitiveResearchKit
<https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share/igtl>`_.


Discussion
**********

Compared to |sawSocketStreamer|_:

* Uses binary OpenIGTLink serialization rather than text JSON — more efficient
  and preserves full floating-point precision
* Requires an OpenIGTLink library on the client side
* Natively integrated into 3D Slicer (no extra plugin needed)

Useful links
============

* `sawOpenIGTLink repository <https://github.com/jhu-saw/sawOpenIGTLink>`_
* `OpenIGTLink protocol <https://openigtlink.org>`_
* `3D Slicer <https://www.slicer.org>`_
* `SlicerROS2 module <https://slicerros2.readthedocs.io>`_
* `sawIntuitiveResearchKit – IGTL examples
  <https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share/igtl>`_
