.. _thread:

Threads
#######

In general, each component owns a thread and can run at its own
frequency.  There are a few notable exceptions:

* There is a single IO component with multiple interfaces (one per arm
  connected).  This is required to:

  * Minimize the number of IOs (inputs/outputs).  For n boards (2 per
    arm controllers in most cases), sequential reads and writes would
    require 2 x n IOs, one read and one write per board.  FireWire
    also allows to broadcast a single write command to multiple
    controllers and read from all controllers in a single message.
    This reduces the number of IOs to a fixed number (~2), whatever
    the number of controllers are used.  This is only possible if a
    single thread manages all the IOs.
  * Avoid simultaneous accesses to the FireWire port from multiple
    threads (FireWire read/write are thread safe but processes can
    hang for a couple seconds).

* The PID components could run in separate threads but this would
  introduce a fair amount of latency since the thread safe
  communication mechanisms in *cisstMultiTask* are based on queues.
  Assuming a 1 millisecond period for both IO and PID, the PID would
  read some cached data (position and velocity) from the IO (between
  0+ and 1 millisecond old) and then request a new effort.  This
  request being queued will be acted on between 0+ and 1 millisecond
  later.  Overall, the time between read and write could be as high as
  2 milliseconds.  Instead, we used the *cisstMultiTask*
  ExecIn/ExecOut feature (see `cisstMultiTask concepts
  <https://github.com/jhu-cisst/cisst/wiki/cisstMultiTask-concepts>`_)
  which allows to attach a component to another.  Effectively, the
  parent thread now runs the child's computation whenever needed.  In
  pseudo code:

  .. code-block:: C++

     IO::Run(void) {
       ReadAllData();
       SaveReadDataInStateTable(); // state tables are used to cache data
       ExecOut(); // trigger PID.Run() for all PID components attached to this IO
       ProcessQueuedCommands(); // dequeue all commands, including those from PID
     }

* Qt manages its own thread(s)
* The ROS bridges (cisst-ros) also use multiple threads

.. figure:: /images/software/dVRK-component-thread-view.png
   :align: center

   Main components and threads for the dVRK software stack
