.. _config-suj:

.. include:: /includes/logic-view-soft-arm.rst

***
SUJ
***

The SUJ configuration files are not documented (no JSON schema) and there is no
configuration file generator. For all SUJs, you will need to copy existing
configuration files and edit them by hand.

There are three types of SUJ for the dVRK: *Classic*, *Si* and *Fixed*.

The configurations files are somewhat similar across the SUJ types but not
exactly the same.  Make sure you start from the correct version.

Classic
=======

Examples can be found in ``dvrk/dvrk_config_jhu/jhu-daVinci``.

 * ``sawRobotIO1394-SUJ.json``: This file is unique to each system since it is
   calibrated for current feedback so, you should keep your version in your configuration file directory 
 * ``suj-ECM-1-2-3.json``: This file contains the DH parameters for all the SUJs
   as well as the calibration results for the potentiometers to joint values
   conversion.  As such, this file is also specific to each system.
 * ``system-SUJ.json``: These files are used with the dVRK system applications.
   If you're creating a configuration file with the SUJs and the actual arms
   (ECM and PSMs), don't forget to set the ``base-frame`` for each active arm.

Si
===

Examples can be found in ``dvrk/dvrk_config_jhu/jhu-daVinci-Si``.

The system file contains one ``SUJ_Si`` arm entry whose ``arm_file``
points to the Si SUJ kinematic configuration, for example:

.. code-block:: json

   {
       "name": "SUJ",
       "type": "SUJ_Si",
       "arm_file": "kinematic/suj-si.json"
   }

The kinematic configuration contains the DH parameters for all the SUJ arms.
For real Si SUJ hardware, each mounted ECM or PSM also has a per-arm IO
calibration file named ``sawRobotIO1394-SUJ-Si-<arm>-<serial>.json``.  These
files contain the ``primary_measured_js`` and ``secondary_measured_js``
scale/offset values used to convert SUJ potentiometer voltages to joint
positions.  They are generated along with the Si arm IO files and updated by
the :ref:`Si SUJ potentiometer calibration <calibration-si-suj-pots>` script.

Fixed
=====

Example: ``dvrk/dvrk_config_jhu/jhu-daVinci/suj.json``.

There is no IO file for Fixed SUJ.

The JSON configuration file doesn't contain any kinematic information (DH,
offsets...). It only contains the transformation (4x4 matrix) between the world
and the base of each active arm.  Said transformations are usually computed by
some kind of hand-eye registration program (see
:ref:`dvrk_camera_registration<dvrk_camera_registration>`).
