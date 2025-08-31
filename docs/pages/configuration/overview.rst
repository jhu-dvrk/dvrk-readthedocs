.. _configuration-overview:

********
Overview
********

The dVRK relies on multiple configurations files.  Some are shared
across systems and some are specific to each arm or site.  For the
site specific configuration files, we provide a configuration
generator that simplify the process, but some files still have to be
created by hand.

.. _configuration-files-types:

Types of files
##############

Site specific files
*******************

* **System**: These files are specific to each site.  They are used by
  the :ref:`dVRK system applications<system>` to define the
  combination of arms you're using as well as a few other parameters
  such as IO parameters (period, port), head sensor used, foot pedal,
  teleoperation component. The most basic system file contains a
  single arm and uses the default options for everything else. The
  single arm system files are :ref:`automatically generated
  <config-generators>` along the arm IO file. The system configuration
  files are parsed by the ``Configure`` method of the
  :ref:`sawIntuititiveResearchKit<system-component>` ``dvrk::system``
  component, they use the JSON file format.  See :ref:`system
  configuration files <config-system>`

* **Arm**: These files specify the configuration of a specific arm
  (e.g. MTML, PSM2...).  They are therefore not shared across dVRK
  sites and their name includes the arm's serial number
  (e.g. ``PSM1-17425.json``).  They use the JSON file format. See
  :ref:`arm configuration files <config-arm>`.

* **IO (input/output):** These files are used by the
  :ref:`sawRobotIO1394<io>` software components.  They specify which
  signals coming in and out of the dVRK controllers a device needs to
  use and how to convert them to something useful.  For example a
  digital input low/high can be converted to a pedal pressed or
  released, an analog input can be converted to a potentiometer position
  in SI units, an analog output can be used to send a desired motor
  current...  IO files use the JSON language and follow the naming
  convention ``sawRobotIO1394-xxxx.json``.

  * There is one IO configuration file per active arm (MTM, PSM and
    ECM).  This file is specific to each arm since it will store the
    results of different calibration steps.  Therefore, its name
    contains the arm's serial number.  For example,
    ``sawRobotIO1394-PSM1-27425.json``. These files are not shared
    across systems.

  * Some of the IO configuration files don't require any calibration
    and can be shared across sites.  For example, IOs used for the
    foot pedals, head sensors, focus controller...

Shared files
************

* **Kinematics**: These files are used by the
  :ref:`sawIntuititiveResearchKit<arms>` arm
  components.  There is one file per arm type.  They are shared across
  systems and use the JSON file format.  The format is defined in the
  *cisst* library *cisstRobot*, class ``robManipulator``.

* **Instrument**: These files are used to store all the parameters
  specific to a given instrument, i.e. kinematic of the last 3 joints,
  jaw angle and torque limits...  These are shared across all dVRK
  sites unless you are creating your :ref:`custom
  instrument<config-custom-instruments>`.  They used the JSON file
  format and the filename follows the naming convention
  ``<TOOL_NAME>_<MODEL_NUMBER>.json``
  (e.g. ``LARGE_NEEDLE_DRIVER_400006.json``).  See :ref:`instrument
  naming<instruments>`.

* **Instruments list**: These are shared files listing all the
  available instruments (aka tools).  Most users will never need to
  modify these unless they need to add a new instrument (e.g. custom-built).
  They use the JSON file format and the documentation is
  generated from a schema:

  * `tool list documentation <../../schemas/dvrk-tool-list.html>`_

* **PID**: These files are used by the
  :ref:`sawControllers<pid>` ``mtsPID`` component.  They
  contain the default `PID
  <https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller>`_
  parameters, use the JSON file format and follow the naming
  convention ``sawControllersPID-<arm>.json``. For example
  ``sawControllersPID-MTMR.json``. Note that the file name doesn't
  contain the arm serial number since these are shared across arms of
  the same type (e.g. ``MTML``, ``ECM``, ``PSM-Si``).


Where are the configuration files
#################################

Site specific files
*******************

For site specific examples, we recommend using the JHU configuration
repository as a template for the files ``CMakeLists.txt``,
``package.xml`` and ``colcon.pkg``:
https://github.com/dvrk-config/dvrk_config_jhu

Assuming a site can have multiple systems, we use subdirectory per
system. Directory names start with the institution name (e.g. jhu for
Johns Hopkins, isi for Intuitive Surgical) and should contain the
system name (e.g. JHU has four systems, an original research kit:
``jhu-dVRK``, a full da Vinci Classic: ``jhu-daVinci``, a system with
a S console and Si patient cart: ``jhu-daVinci-Si`` and a spare PSM
Si: ``jhu-dVRK-Si``).

We strongly encourage each dVRK site to use their own configuration
repository under https://github.com/dvrk-config.  If you need a new
repository or access to an existing one, contact the dVRK maintainers.

Each directory should contain:

  * your IO configuration files, ``sawRobotIO1394-xxxxx.json``, for
    each arm identified by its number.  You should also store the
    original ``.cal`` files provided by Intuitive Surgical since they
    are needed to re-generate the IO JSON files (for Classic arms only)
  * your arms configuration files
  * your system configuration files since these refer to your system
    specific IO configuration files

It is recommended to use tags or branches to maintain your
configurations files for each version of the dVRK.  For example, files
for the dVRK 2.4 are different from 2.3 but you might need both at a
given time.

Before checking in files, you should clean your configuration
directory, i.e. remove the temporary and log files (see
:ref:`dvrk-remove-logs.py <remove-logs>`).

Site specific directories should **NOT** contain any of the shared
files.  If you make a copy of the shared file in your directory, it
will be used in place of the default one.  dVRK applications use a
search path that includes the directory containing the system JSON
file, the current working directory and then the dVRK shared
directory.  The ``cisstLog-xxx.txt`` file generated when you run the
dVRK code should log the path of each configuration file loaded.

Shared files
************

All the shared files are under the ``/share`` directory of the main
dVRK repository: |sawIntuitiveResearchKit|.

The subdirectories are:

* ``io``: IO files for foot pedals, head sensors, focus controller...
* ``pid``: default PID configuration files
* ``kinematics``: kinematics (DH and maximum torques) for all dVRK
  arms
* ``tools``: all instrument definition files as well as main list
  (``index.json``)
* ``arm``: definition of arm that are not site specific, mostly
  simulated arms
* ``system``: definition of systems using no site specific arms,
  mostly for simulation

How to create the configuration files
#####################################

In most cases, users don't have to create the instrument, kinematic
and PID configuration files since these are shared.

For the site specific files, we provide a configuration generator
(Python based) that will generate a blank IO configuration file for an
arm, as well as the arm configuration file and a system configuration
file for said arm.  See :ref:`configuration generators
<config-generators>`.

We are working on a system configuration file generator but
meanwhile, you will have to start from existing configuration files
and edit by hand.  Since the JHU configurations files are usually
up-to-date, we recommend to look at
https://github.com/dvrk-config/dvrk_config_jhu.

.. caution::

   If you edit a configuration file by hand, pay close attention to
   units as we used different ones in different sections!  Older files
   might use millimeters and degrees.  Most other files use SI units,
   i.e. meters and radians.

.. caution::

   For Si PSMs and ECMs, the serial number is used to identify
   calibration files. If you ever have to edit configuration files for
   these arms by hand, make sure the serial numbers are correct!
