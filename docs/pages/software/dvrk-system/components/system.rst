.. _system-component:

System
######

The dVRK system component is managing all the other dVRK components. Based on
the :ref:`system configuration file <config-system>`, it will:

* Build a search path based on the current working directory, the directory
  containing the system configuration file and the main dVRK shared directory
  (``src/cisst-saw/sawIntuitiveResearchKit/share``). This search path is used to
  locate the "sub" configuration files used by all the dVRK components.
* Dynamically create and configure all the :ref:`IO components <io>`. If extra
  configuration files are provided for the IO, the system component will attempt
  to locate them and use them to configure the IO instance.
* For each arm defined in the configuration file, dynamically create and
  configure the appropriate instances of :ref:`dVRK arms <arms>` (MTM, PSM, ECM,
  SUJ...).
* If the arm requires it, a :ref:`PID component <pid>` will also be created and
  configured.
* If the arm is using a dVRK IO (e.g. not a simulated arm), the system component
  will locate the corresponding IO configuration file corresponding to the arm,
  and use it to configure the IO component.
* For each console defined in the configuration file, a console object is
  created (C++ class not a cisstMultiTask component). All the console objects
  are internal to the system, Their functionalities are exposed using interfaces
  provided by the system component.
* For each console, the components required to provide the user events *operator
  present*, *clutch* and *camera* are dynamically created.
* For each teleoperation (:ref:`PSM <teleop-psm>` or :ref:`ECM <teleop-ecm>`)
  declared in the *consoles* section of the system configuration file, the
  system creates and configures the required components.
* Finally, the system will connect all the required and provided interfaces.

There are also two special dVRK system classes: system Qt component and system
ROS bridge. They dynamically create, configure and connect the dVRK Qt widgets
or dVRK ROS bridges (ROS1 or ROS2) based on the system's configuration.

It is possible to start the :ref:`dVRK system <system>` without the ROS bridge
with *sawIntuitiveResearchKitSystem* or without the Qt interface (``-t`` command
line option).

Once the dVRK application and configured, the system components manages the
overall state of the dVRK:

* Power on, off, home...
* Collect all messages
* Enable, disable teleoperation per console
* Manage which teleoperation component should be enabled based on which ones are selected
* Propagate the SUJ reference frames if needed


Console
#######

On a clinical da Vinci system, there are usually 3 PSMs and 2 MTMs.  The user
has to select which PSMs should be teleoperated at a given time (selected) and
which one should be left alone (unselected).  The clinical system uses a menu
(or buttons) on the console to set the configuration (e.g. MTMR will drive PSM1
and PSM3 while MTML will drive PSM2).  Then the operator can use a foot pedal to
toggle the PSMs to the MTM configured to drive 2 PSMs.  For the da Vinci Classic
and S, the operator had to do a "quick tap" on the clutch pedal or use the
console's buttons. The dVRK also implements the quick tap to toggle between PSMs
for one MTM.

It is also possible to have two surgeon's consoles. In this case, the system has
the following arms: SUJ, ECM, PSM1, PSM2 and PSM3 on the patient's side. Then we
have MTML1 and MTMR1 for the first console and MTML2 and MTMR2 for the second
console.  At minima, each console needs two MTMs as well as 3 inputs for
*operator present*, *clutch* and *camera*. There are many possible
teleoperations:

* Any MTM can control any PSM (4x3)
* MTML1 and MTMR1 or MTML2 and MTMR2 can control the ECM (2)

Not all 14 combinations make sense based on how the PSMs are configured on the
patient's side.  But it makes sense to be able to toggle which arms are
controlled and by whom. For the dVRK, all the desired sets (``MTML1_PSM2``,
``MTML1_MTMR1_ECM``...) have to be declared in the system configuration file. An
instance of each teleoperation is also created but, they are not all selected at
the same time.  The system component prevents conflicts (any arm used by two
teleoperation components) by automatically unselecting one of the
teleoperations. Users can select and unselect teleoperation components using the
graphical user interface or ROS topics.

Finally, the dVRK allows some configurations not supported on the Clinical
system such as two ECMs on a patient's cart.
