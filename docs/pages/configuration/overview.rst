.. _configuration-overview:

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
  :ref:`*sawRobotIO1394*<io>` software components.  They specify
  which signals coming in and out of the dVRK controllers a device needs
  to use and how to convert them to something useful.  For example a
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
  :ref:`sawControllers<pid>` ``mtsPID`` component.  They
  contain the default `PID
  <https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller>`_
  parameters, use the JSON file format and follow the naming
  convention ``sawControllersPID-<arm>.json``. For example
  ``sawControllersPID-MTMR.json``. Note that the file name doesn't
  contain the arm serial number since these are shared across arms of
  the same type (e.g. ``MTML``, ``ECM``, ``PSM-Si``).

* **Kinematics**: These files are used by the
  :ref:`sawIntuititiveResearchKit<arms>` arm
  components.  There is one file per arm type.  They are shared across
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

* **Instruments list**: These are shared files listing all the
  available instruments (aka tools).  Most users will never need to
  modify these unless they need to add a new instrument (e.g. custom
  built).  They use the JSON file format and the documentation is
  generated from a schema:

  * `tool list documentation <../../_static/schemas/dvrk-tool-list.html>`_

* **Arm**: These files specify the configuration of a specific arm.
  They are therefore not shared across dVRK sites and their name
  includes the arm's serial number (e.g. ``PSM1-17425.json``).  They
  use the JSON file format and the documentation is generated from a schema:

  * `MTM documentation <../../_static/schemas/dvrk-mtm.html>`_
  * `PSM documentation <../../_static/schemas/dvrk-psm.html>`_
  * `ECM documentation <../../_static/schemas/dvrk-ecm.html>`_

* **Console**: These files are specific to each site.  They are used
  by the :ref:`sawIntuititiveResearchKit<console>`
  ``mtsIntuitiveResearchKitConsole`` component to define the
  combination of arms you're using as well as a few other parameters
  such as IO parameters (period, port), head sensor used, foot pedal,
  teleoperation component.  The most basic console file contains a
  single arm and uses the default options for everything else.  The
  single arm console files are :ref:`automatically
  generated <config-generators>` along the arm IO file.  The console
  configuration files use the JSON file format and the documentation
  is generated from a schema:

  * `console documentation <../../_static/schemas/dvrk-console.html>`_


Where are the configuration files
#################################

Shared files
************

All the shared files are under the ``/share`` directory of the main
dVRK repository:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share

The sub-directories are:

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

For site specific examples, we recommend to use the JHU configuration
repository as a template for the files ``CMakeLists.txt`` and
``package.xml``: https://github.com/dvrk-config/dvrk_config_jhu

Assuming a site can have multiple systems, we use sub-directory per
system. Directory names start with the institution name (e.g. jhu for
Johns Hopkins, isi for Intuitive Surgical) and should contain the
system name (e.g. JHU has two systems, a research kit: ``jhu-dVRK``,
and a full da Vinci: ``jhu-daVinci``).

We strongly encourage each dVRK site to use their own configuration
repository under https://github.com/dvrk-config.  If you need a new
repository or access to an existing one, contact the dVRK maintainers.

Each directory should contain:

  * your IO configuration files, ``sawRobotIO1394-xxxxx.xml``, for
    each arm identified by its number.  You should also store the
    original ``.cal`` files provided by Intuitive Surgical since they
    are needed to re-generate the IO XML files (for Classic arms only)
  * your arms configuration files
  * your console configuration files since these refer to your system
    specific IO configuration files

Site specific directories should **NOT** contain any of the shared
files.  If you make a copy of the shared file in your directory, it
will be used in place of the default one.  dVRK applications use a
search path that includes the directory containing the console JSON
file, the current working directory and then the dVRK shared
directory.  The ``cisstLog-xxx.txt`` file generated when you run the
dVRK code should log the path of each configuration file loaded.

How to create the configuration files
#####################################

In most cases, users don't have to create the instrument, kinematic
and PID configuration files since these are shared.

For the site specific files, we provide a configuration generator
(Python based) that will generate a blank IO configuration file for an
arm, as well as the arm configuration file and a console configuration
file for said arm.  See :ref:`configuration generators
<config-generators>`.

We are working on a console configuration file generator but
meanwhile, you will have to start from existing configuration files
and edit by hand.  Since the JHU configurations files are usually
up-to-date, we recommend to look at
https://github.com/dvrk-config/dvrk_config_jhu.

.. caution::

   If you edit a configuration file by hand, pay close attention to
   units as we used different ones in different sections!  Older files
   might use millimeters and degrees.  Most other files use SI units,
   i.e. meters and radians.
