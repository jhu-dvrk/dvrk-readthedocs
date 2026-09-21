.. include:: /includes/logic-view-soft-bridge.rst

.. _devel-bridge-socket-streamer:

.. _udp-json:

sawSocketStreamer
#################

Description
***********

|sawSocketStreamer|_ serializes cisst/SAW component commands and events as
**JSON messages over UDP sockets**. Because it has no dependency on ROS, it
runs on any operating system — Linux, macOS, Windows — making it the bridge
of choice for cross-platform integrations.

Typical use cases:

* Quick prototyping without a full ROS installation
* Unity / HoloLens applications running on Windows
* Any non-Ubuntu OS where ROS is unavailable or impractical
* Lightweight scripts that need only a few dVRK topics

All messages are text-based JSON, so they are human-readable and easy to
parse with any language that has a JSON library. The main trade-offs are
slightly higher computing cost and reduced floating-point precision compared
to binary formats, but in practice these rarely matter.


How to
******

Installation
============

Clone the repository into your workspace:

.. code-block:: bash

   cd ~/ros2_ws/src/cisst-saw   # adjust to your workspace
   git clone https://github.com/jhu-saw/sawSocketStreamer

Build alongside the dVRK:

.. code-block:: bash

   colcon build   # ROS 2
   # or: catkin build  (ROS 1)
   # or: cmake / make  (no ROS)

Configuration files
===================

You need at least two configuration files:

1. **Component manager file** (passed with ``-m``): tells |cisstMultiTask|_ to
   dynamically create and connect the ``sawSocketStreamer`` component.
2. **Streamer configuration file**: lists which commands (read/write) and events
   to expose, together with their payload types and socket settings.

Multiple ``-m`` options can be combined to load several bridges simultaneously.

Running
=======

.. code-block:: bash

   ros2 run dvrk_robot dvrk_system \
       -j your_system_config.json \
       -m your_socket_streamer_manager.json

Examples
========

Configuration files, usage examples and a simple Python client are in the
``share/socket-streamer`` directory of
`sawIntuitiveResearchKit
<https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share/socket-streamer>`_.


Discussion
**********

Pros
====

* No ROS dependency — works on Windows, macOS, embedded Linux
* Human-readable JSON simplifies debugging (e.g. with ``netcat``)
* Can run alongside any combination of other bridges

Cons
====

* Text serialization is slower and less compact than binary formats
* Floating-point values lose some precision when serialized as JSON strings
* UDP is unordered and unreliable — suitable for state streaming, not for
  guaranteed command delivery

Useful links
============

* `sawSocketStreamer repository <https://github.com/jhu-saw/sawSocketStreamer>`_
* `sawIntuitiveResearchKit – socket-streamer examples
  <https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share/socket-streamer>`_
