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

Example: ``dvrk/dvrk_config_jhu/jhu-daVinci/suj-si.json``.

There is no IO file for the Si SUJ.

The JSON configuration file contains the DH parameters for all the SUJs as well
as the calibration results for the potentiometers to joint values conversion.

Fixed
=====

Example: ``dvrk/dvrk_config_jhu/jhu-daVinci/suj.json``.

There is no IO file for Fixed SUJ.

The JSON configuration file doesn't contain any kinematic information (DH,
offsets...). It only contains the transformation (4x4 matrix) between the world
and the base of each active arm.  Said transformations are usually computed by
some kind of hand-eye registration program (see
:ref:`dvrk_camera_registration<dvrk_camera_registration>`).