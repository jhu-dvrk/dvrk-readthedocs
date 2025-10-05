.. _middleware:

.. include:: /includes/logic-view-soft-bridge.rst

*******
Bridges
*******

Like any other cisst/SAW component, the dVRK can use different types of
"bridges" to interface with some commonly used middleware. The default
middleware is ROS (either version 1 for Ubuntu up to 20.04 or ROS 2 for Ubuntu
20.04 or newer). The dVRK comes with a :ref:`ROS node <system>` statically
linked to the ROS bridges, package ``dvrk_robot``, node ``dvrk_system``.

For all other middleware supported (OpenIGTLink, UDP with JSON), we use dynamic
loading and configuration files. Most binaries distributed along the dVRK accept
the ``-m`` command line option to create, configure and connect extra bridges.
These are not exclusive, One can perfectly have a single dVRK ROS node with ROS
topics, an OpenIGTL connection for Slicer on a Mac and a plain UDP socket
sending JSON string to a HoloLens running Windows (see also
:ref:`middleware<devel-middleware>` in development options).




