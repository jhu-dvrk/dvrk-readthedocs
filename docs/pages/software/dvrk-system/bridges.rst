.. _middleware:

.. include:: /includes/logic-view-soft-bridge.rst

*******
Bridges
*******

Like any other cisst/SAW component, the dVRK can use different types of
"bridges" to interface with commonly used middleware. The default middleware is
ROS (either version 1 for Ubuntu up to 20.04 or ROS 2 for Ubuntu 20.04 or
newer). The dVRK comes with a :ref:`ROS node <system>` statically linked to the
ROS bridges, package ``dvrk_robot``, node ``dvrk_system``.

For all other supported middleware (OpenIGTLink, UDP with JSON), we use dynamic
loading and configuration files. Most binaries distributed with the dVRK accept
the ``-m`` command line option to create, configure and connect extra bridges.
These are not exclusive: one can perfectly have a single dVRK ROS node with ROS
topics, an OpenIGTLink connection for Slicer on a Mac and a plain UDP socket
sending JSON to a HoloLens running Windows.

See also :ref:`bridge usage <devel-bridges>` in the development section.


.. _arch-bridge-ros:

ROS
===

The ROS bridge uses `cisst-ros <https://github.com/jhu-cisst/cisst-ros>`_, a
generic bridge library that auto-discovers all provided and required interfaces
of any |cisstMultiTask|_ component and exposes them as ROS topics and services.
Because the bridge is self-configuring, adding a new interface to a dVRK
component automatically makes it available over ROS without any extra bridge
code.

The dVRK system can run without ROS on any operating system, using the
``sawIntuitiveResearchKitSystem`` executable. However, most users run with ROS
since the built-in ``dvrk_robot dvrk_system`` executable and the surrounding
ROS ecosystem (visualization, recording, Python/Matlab clients) cover the vast
majority of use cases.

See :ref:`ROS bridge usage <devel-bridge-ros>` for details on running and
configuring the ROS bridge.


.. _arch-bridge-socket-streamer:

sawSocketStreamer
=================

|sawSocketStreamer|_ is a dynamically loadable SAW component that serializes
commands and events as JSON messages over UDP sockets. Because it does not
depend on ROS, it runs on any platform — Linux, macOS, Windows — making it
ideal for quick prototyping and cross-platform integrations such as Unity,
HoloLens, or any custom application that can parse JSON.

The user provides a configuration file listing which commands and events to
expose; the bridge then connects to the running dVRK components and starts
streaming. The JSON format is human-readable, which simplifies debugging even
at the cost of slightly higher serialization overhead compared to binary
formats.

See :ref:`sawSocketStreamer usage <devel-bridge-socket-streamer>` for
details on configuration and examples.


.. _arch-bridge-igtlink:

sawOpenIGTLink
==============

|sawOpenIGTLink|_ follows the same dynamically loadable pattern as
|sawSocketStreamer|_, but uses the `OpenIGTLink protocol
<https://openigtlink.org>`_ instead of UDP/JSON. OpenIGTLink is a well-known
standard in the surgical robotics and medical imaging community, and is natively
supported by `3D Slicer <https://www.slicer.org>`_.

The main audience is researchers using 3D Slicer for visualization, image
guidance, or registration tasks alongside the dVRK. As with the socket
streamer, the user provides a configuration file to select which interfaces to
expose.

.. note::

   The dVRK developers have also collaborated on the `SlicerROS2
   <https://slicerros2.readthedocs.io>`_ module, which provides a more complete
   ROS 2 integration for 3D Slicer users who are already working within a ROS 2
   environment.

See :ref:`sawOpenIGTLink usage <devel-bridge-igtlink>` for details on
configuration and examples.
