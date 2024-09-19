********
Overview
********

The dVRK relies on multiple configurations files.  Some are shared
across systems and some are specific to each arm or site.  For the
site specific configuration files, we provide a configuration
generator that simplify the process but some files have to be create
by hand.

Types of files
##############

* **IO (input/output):** These files are used by the
  :ref:`*sawRobotIO1394*<sawrobotio>` software components.  They specify
  which signals coming in and out of the dVRK controllers a device needs
  to use and how to convert them to something useful.  For exemple a
  digital input low/high can be converted to a pedal pressed or
  released, an analog input can be converted to a potentiometer position
  in SI units, an analog output can be used to send a desired motor
  current...  IO files use the XML language and follow the naming
  convention ``sawRobotIO1394-xxxx.xml``.

  * There is one IO configuration file per active arm (MTM, PSM and
    ECM).  This file is specific to each arm since it will store the
    results of different calibration steps.  Therefore its name
    contains the arm's serial number.  For example,
    ``sawRobotIO1394-PSM1-27425.xml``. These files are not shared
    across systems.

  * Some of the IO configuration files don't require any calibration
    and can be shared across sites.  For example, IOs used for the
    foot pedals, head sensors, focus controller...

* **PID**: These files are used by the
  :ref:`sawControllers<sawcontrollers>` ``mtsPID`` component.  They
  contain the default `PID
  <https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller>`_
  parameters, use the JSON file format and follow the naming
  convention ``sawControllersPID-<arm>.json``. For example
  ``sawControllersPID-MTMR.json``. Note that the file name doesn't
  contain the arm serial number since these are shared across arms of
  the same type (e.g. ``MTML``, ``ECM``, ``PSM-Si``).

* **Kinematics**: These files are used by the
  :ref:`sawIntuititiveResearchKit<sawintuitiveresearchkit>` arm
  components.  There is one file per arm type.  They are shared accross
  systems and use the JSON file format.  The format is defined in the
  *cisst* library *cisstRobot*, class ``robManipulator``.

* **Instrument**: These files are used to store all the parameters
  specific to a given instrument, i.e. kinematic of the last 3 joints,
  jaw angle and torque limits...  These are shared across all dVRK
  sites unless you are creating your :ref:`custom
  instrument<custom-instrument>`.  They used the JSON file format and
  the filename follows the naming convention
  ``<TOOL_NAME>_<MODEL_NUMBER>.json``
  (e.g. ``LARGE_NEEDLE_DRIVER_400006.json``).  See :ref:`instrument
  naming<instrument-naming>`.

* **Intruments list**: These are shared files listing all the
  available instruments (aka tools).  Most users will never need to
  modify these unless they need to add a new instrument (e.g. custom
  built).  They use the JSON file format and the documentation is
  generated from a schema:
  https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.1/dvrk-tool-list.html

* **Arm**: These files specify the configuration of a specific arm.
  They are therefore not shared across dVRK sites and their name
  includes the arm's serial number (e.g. ``PSM1-17425.json``).  They
  use the JSON file format and the documentation is generated from a schema:

  * MTM: https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.2/dvrk-mtm.html
  * PSM: https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.2/dvrk-psm.html
  * ECM: https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.2/dvrk-ecm.html

* **Console**: These files are specific to each site.  They are used
  by the :ref:`sawIntuititiveResearchKit<sawintuititiveresearchkit>`
  ``mtsIntuitiveResearchKitConsole`` component to define the
  combination of arms you're using as well as a few other parameters
  such as IO parameters (period, port), head sensor used, foot pedal,
  teleoperation component.  The most basic console file contains a
  single arm and uses the default options for everything else.  The
  single arm files are :ref:`automatically
  generated<config-generators>`.  The console configuration files use
  the JSON file format and the documentation is generated from a
  schema:
  https://dvrk.lcsr.jhu.edu/documentation/schemas/v2.2/dvrk-console.html


Where are the configuration files
#################################

Shared files
************

All the shared files are under the ``/shared`` directory of the main
dVRK repository:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share

The subdirectories are:

* ``io``: IO files for foot pedals, head sensors and focus controller
* ``pid``: default PID configuration files
* ``kinematics``: kinematics (DH and maximum torques) for all dVRK
  arms
* ``tools``: all instrument definition files as well as main list
  (``index.json``)
* ``arm``: definition of arm that are not site specific, mostly
  simulated arms
* ``console``: definition of console using no site specific arms,
  mostly for simulation

Site specific files
*******************

* For site specific examples, we recommend to use the JHU configuration repository: https://github.com/dvrk-config/dvrk_config_jhu


How to create the configuration files
#####################################

To get started, you will need to generate your [IO XML configuration file](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig).  The Python configuration generator will also create the Arm JSON file as well as a simple Console JSON file.

**Pay close attention to units as we used different ones in different sections!**
