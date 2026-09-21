.. _devel-generic-components:

.. _devel-components:

******************
Generic components
******************

Instead of using a middleware bridge and implementing your application logic in
a separate process, you can add cisst/SAW components directly to the running
dVRK process. The main advantage is performance: communication between
components uses thread-safe queues with no serialization or network overhead.

The whole dVRK system is built this way — IO, PID, arm logic, teleoperation,
Qt widgets and ROS bridges are all |cisstMultiTask|_ components. You can keep
the existing components and add your own, or replace a compatible component
with an alternative (plugin).

See also |cisstMultiTask|_ for concepts and the tutorial.

.. toctree::

   components/IRE
   components/collectors
   components/plugins
