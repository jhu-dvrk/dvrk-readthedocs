.. _applications:

************
Applications
************

For all applications, you will need to source your ROS workspace's
``setup.bash``.  For ROS1, use ``source
<workspace>/devel/setup.bash``.  For ROS2, use ``source
<workspace>install/setup.bash``.

Some applications are ROS nodes (or ROS launch files), these can be
started using ``ros2 run <package> <application>``.  Some are not ROS
nodes and can be launched directly from a terminal like any other
executable.

.. toctree::

   applications/console
   applications/debug
   applications/calibration
   applications/utilities
