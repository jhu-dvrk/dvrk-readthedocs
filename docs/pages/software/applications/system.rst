.. _system:

System
#######

Overview
********

The dVRK system application is the main way to interact with a dVRK
system. It comes in two versions:

* ``dvrk_robot dvrk_system``

  * C+ application with a Qt based GUI and ROS interface, it depends on cisst/SAW
  * Compatible with all dVRK controllers
  * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/ros/dvrk_robot

* ``sawIntuitiveResearchKitSystem``

  * C+ application with a Qt based GUI, it depends on cisst/SAW
  * Compatible with all dVRK controllers (including for Windows/MacOS over UDP)
  * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications


For ROS, start the node ``dvrk_system`` from the package
``dvrk_robot``.

::

   ros2 run dvrk_robot dvrk_system

The non-ROS version is an executable found in the dVRK path.

::

   sawIntuitiveResearchKitSystem


Command line options
********************

``dvrk_system`` supports all the command line options of
``sawIntuitiveResearchKitSystem`` as well as options to
configure the ROS publish rates and some optional ROS topics.


Shared options
==============

``-j <value>, --json-config <value>``
  *json configuration file (required)*

  Full path to the :ref:`system configuration
  file<system-configuration-file>`. This is the only required
  option.

``-t, --text-only``
  *text only interface, do not create Qt widgets
  (optional)*

  This is rarely needed, the dVRK system
  would have to be controlled and monitored over some kind of
  middleware (for example ROS, OpenIGTL...).

``-c <value>, --collection-config <value>``
  *json configuration file for data collection using cisstMultiTask state table collector (optional)*

  See examples of configurations files in the ``shared/collection``
  directory. This feature predates the dVRK ROS integration. Users
  should look into ROS bags for data collection.

``-C, --calibration-mode``
  *run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing. This mode should only be used when calibrating your potentiometers. (optional)*

  This command line option is required for many :ref:`calibration
  steps <calibration>`. It shouldn't be used on a regular basis since
  this disables some safety checks.

``-e <value>, --embedded-python <value>``
  *start an embedded Python shell to access all dVRK software components (optional)*

  This will start an embedded Python interpreter along all the dVRK software
  components. The embedded interpreter can be either IPython (``IRE_IPYTHON``
  **recommended**) or wxPython (``IRE_WXPYTHON`` **experimental**).  See also
  :ref:`development options<components-IRE>`.

``-m, --component-manager``
  *JSON files to configure component manager (optional)*

  See :ref:`dynamic components
  <system-dynamic-components>`.

``-S <value>, --qt-style <value>``
  *Qt style, use this option with a random name to see available styles (optional)*

  See :ref:`widget customization<widgets-customization>`.

``-D, --dark-mode``
  *replaces the default Qt palette with darker colors (optional)*


ROS extra options
=================

``-p <value>, --ros-period <value>``
  *period in seconds to read all arms/teleop components and publish (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the arm component's period (optional)*

  When you're using :ref:`ROS as middleware <bridge-ros>`, publishers for events
  and subscribers for commands are processed in a fast thread, processing
  messages as fast as possible. For all the robot's state topics (e.g. joint,
  kinematic... aka ``measured_js``, ``setpoint_cp``...) there is a periodic
  thread publishing the state at a constant rate. If your application can
  process message at a higher rate, you can increase the period used to publish
  to the robot's state.

``-P <value>, --tf-ros-period <value>``
  *period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period (optional)*

``-s, --suj-voltages``
  *add ROS topics for SUJ voltages (optional)*

``-I, --pid-topics-read-only``
  *add some extra publishers to monitor PID state (optional)*

  This option is mostly for debugging purposes.

``-J, --pid-topics-read-write``
  *add all PID topics (use with caution!) (optional)*

  This option is dangerous as it allows you to send commands directly to the PID components.

``-K, --io-topics-read-only``
  *add some extra publishers to monitor IO state (optional)*

  This option is used for both debugging and :ref:`calibration<calibration>`.

``-L, --io-topics-read-write``
  *add all IO topics (use with caution!) (optional)*

  This option is dangerous as it allows you to send commands directly to the IO components.
  
.. _system-configuration-file:

Configuration file
******************

See section *System* in :ref:`configuration files<config-system>`.


.. _system-dynamic-components:

Dynamic components
******************

The command line option ``-m`` can be used to load one or more
configuration files for the cisst component manager.  One can also use
the field ``"component-manager": {}`` in the dVRK system
configuration file itself.

This allows users to dynamically load and connect custom components
such as:

* :ref:`different middleware bridges<devel-middleware>` (OpenIGTLink,
  plain UDP sockets)

* custom applications/components (optionally with widgets and ROS
  bridges), either with :ref:`components derived from existing
  ones<components-derived>` or :ref:`generic ones<components-generic>`

* :ref:`alternative devices<components-alternative-hardware>` for the
  MTM, PSM, foot pedals, head sensor...

The full syntax for the *cisstMultiTask* component manager
configuration files is documented in this `JSON schema
<../../../schemas/cisst-component-manager.html>`_.
