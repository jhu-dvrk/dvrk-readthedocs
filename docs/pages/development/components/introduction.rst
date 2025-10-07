.. include:: /includes/logic-view-console.rst

Introduction
############

Instead of using a middleware and implement the end-user application's
logic in a separate process, you can also add cisst/SAW components,
either existing ones or your own, to the dVRK system.

The main advantage of this approach is performance, i.e. the communication
between components doesn't require any serialization/de-serialization nor
sockets.  *cisstMultiTask* also provides non-blocking and thread-safe
communication mechanisms between threads, so you can take advantage of modern
CPUs with multi-cores.

The whole dVRK system is built using components, the core system, the logic
components for IOs, PID, teleoperation and arms, GUI and ROS bridges.  One can
replace an existing component with another as long as their interfaces are
compatible (plugin).  A plugin can be an existing cisst/SAW component or a
custom one, either derived from an existing one of implemented from scratch.

Finally, one can keep the existing components and add more. This is in a way the
mechanism we use to add the ROS (1 and 2) bridges and Qt interface to the core
logic.  One can also add more middleware bridges such as |sawSocketStreamer|_
and |sawOpenIGTLink|_, a Python embedded interpreter or components designed for
data collection.