.. _config-generators:

Configuration generators
########################

.. _io-config-generator-use:

Arm IO, arm and arm system
***************************

To get started, you should use the Python based application
``dvrk-io-config-generator.py``.  The script should be in your path after
your build and source your ``devel/setup.bash`` (ROS 1) or
``install/setup.bash`` (ROS 2).

This script will generate a "blank" IO XML configuration file for the
arm identified by its :ref:`serial number<serial-number>`.  This file
will be updated by the different calibration steps.  The script also
generates a sample arm configuration file as well as a system file
with a single arm.

The script has 6 options:

* *arm name*: MTMR, MTML, PSM1, PSM2, PSM3 or ECM.
* *generation*: either Classic or Si.  As of 2024, all MTMs are
  Classic.  The PSMs and ECMs can be either Classic or Si (:ref:`PSM
  Classic <psm>`, :ref:`ECM Classic <ecm>`, :ref:`PSM Si <psm-si>` and
  :ref:`ECM Si <ecm-si>`).
* *hardware*: this depends on the controllers you have.  All
  controllers for Classic arms delivered before 2022 are ``QLA1``
  based.  All controllers for Classic arms delivered after 2022 are
  ``DQLA`` based.  All controllers for Si arms (PSMs and ECMs) are
  ``dRAC`` based.  The hardware type should match the output of
  :ref:`qladisp <qladisp>`.
* *calibration file*: also referred as ``.cal`` file. For all Classic
  arms (first generation PSMs and ECMs as well as all MTMs currently
  supported), you will need a ``.cal`` file.  This file contains the
  latest calibration performed by Intuitive Surgical.  You will need
  to reach out to Intuitive Surgical to get these files.  Please save
  them along your dVRK configuration files for future use.
* *serial number*: you can find the serial number on the arm itself
  (aka `trk id`).  The serial number doesn't need to be specified for
  Classic arms since it can be found in the `.cal` file, but you need
  to make sure it matches the number on the arm you're using.
* *port*: port used to connect to the dVRK controllers. It is optional and the default is ``fw``.  If you are using FireWire between the controllers and Ethernet to connect between the PC and the first dVRK controller, use ``-p udpfw``.

There are only 3 possible combinations of generations and hardware/controller type:

* For a Classic arm with a QLA1 based controller (use :ref:`qladisp
  <qladisp>` to find the controller type), you would use:

  .. code-block:: bash

     dvrk-io-config-generator.py -a MTMR -g Classic -H QLA1 -c m23456.cal

* For a Classic arm with a DQLA based controller, you would use:

  .. code-block:: bash

     dvrk-io-config-generator.py -a MTMR -g Classic -H DQLA -c m23456.cal

* For an Si arm, the controller is always dRA1 based, so you need to use:

  .. code-block:: bash

     dvrk-io-config-generator.py -a PSM1 -g Si -H dRA1 -s 123456

.. important::
   
   Once you've generated your arm's configuration files, you will need
   to perform all the :ref:`calibration steps <calibration>` based on
   the arm's type and generation!

System configuration generator
******************************

Future feature!
