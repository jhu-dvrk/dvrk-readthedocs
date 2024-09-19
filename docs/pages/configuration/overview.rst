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
  :ref:`sawRobotIO1394` software components.  They specify which signals
  coming in and out of the dVRK controllers a device need tos use and
  how to convert these to something useful.  For exemple a digital input
  low/high can be converted to a pedal pressed or release, an analog
  input can be converted to a potentiometer position in SI units, an
  analog output can be used to send a desired motor current...  IO files
  use the XML language and follow the naming convention
  ``sawRobotIO1394-xxxx.xml``.

  * There is one IO configuration file per active arm
    (MTM, PSM and ECM).  This file is specific to each arm since it
    will store the results of different calibrations steps.  Therefore
    it's name contains the arm's serial number.  For example,
    ``sawRobotIO1394-PSM1-27425.xml``. These files are not shared
    across systems.

  * Some of the IO configuration files don't require any calibration
    and can be shared across sites.  For example, IOs used for the
    foot pedals, head sensors, focus controller...

* **PID**: These files are used by the ``sawControllers`` ``mtsPID``
  component.  They contain the default `PID
  <https://en.wikipedia.org/wiki/Proportional-integral-derivative_controller>`_
  gains.  These files use the JSON format and follow the naming
  convention ``sawControllersPID-<arm>.json``. For example
  ``sawControllersPID-MTMR.json``. Note that the file name doesn't
  contain the arm serial number since these are shared across arms of
  the same type (e.g. ``MTML``, ``ECM``, ``PSM-Si``).

* **Kinematics**: provided with the dVRK code (shared)

* Instrument: provided with the dVRK code (shared)

* Arm: specific to each arm

* Console: specific to the combination of arms you're using

  
Where are the configuration files
#################################

Shared files
************

You can find examples of configuration files in the `shared` directory:
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/share

Site specific files
*******************

* For site specific examples, we recommend to use the JHU configuration repository: https://github.com/dvrk-config/dvrk_config_jhu
  
How to create the configuration files
#####################################

To get started, you will need to generate your [IO XML configuration file](/jhu-dvrk/sawIntuitiveResearchKit/wiki/XMLConfig).  The Python configuration generator will also create the Arm JSON file as well as a simple Console JSON file.

**Pay close attention to units as we used different ones in different sections!**
