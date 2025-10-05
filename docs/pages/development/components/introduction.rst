.. include:: /includes/logic-view-console.rst

Introduction
############

Instead of using a middleware and implement the end-user application's
logic in a separate process, you can also add cisst/SAW components,
either existing ones or your own, to the dVRK system.  This is
actually the mechanism we use for the ROS (1 and 2),
|sawSocketStreamer|_ and |sawOpenIGTLink|_ described above.  The main
advantage of this approach is performance, i.e. the communication
between components doesn't require any serialization/de-serialization
nor sockets.  *cisstMultiTask* also provides non-blocking and
thread-safe communication mechanisms between threads, so you can take
advantage of modern CPUs with multi-cores.
