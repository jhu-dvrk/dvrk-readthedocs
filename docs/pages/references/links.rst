*****
Links
*****

See also :ref:`acronyms<references_acronyms>`.

Intuitive Surgical
##################

Sites
*****

* `Intuitive Surgical Inc <https://www.intuitive.com/>`_
* `Intuitive Foundation <http://www.intuitive-foundation.org/dvrk/>`_ (dVRK hardware applications)
* `Intuitive Research wiki for the da Vinci Research Kit community <https://research.intusurg.com/>`_ (account required)

Documents
*********

* User manuals

  * :download:`da Vinci Standard <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-classic-user-manual.pdf>`
  * :download:`da Vinci Si <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-si-user-manual.pdf>`


* Instrument catalogs

  * :download:`Standard/S (2011) <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-classic-s-si-instrument-accessory-catalog-08-2011.pdf>`
  * :download:`S/Si (2015) <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-s-si-instrument-accessory-catalog-11-2015.pdf>`
  * :download:`X/Xi (2019) <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-x-xi-instrument-accessory-catalog.pdf>`


Open source
###########

General
*******

* `dVRK Logo <https://github.com/jhu-dvrk/dvrk-logo>`_ (CAD, STL and png)
* `Community publications <https://github.com/jhu-dvrk/community-publications>`_

Mechatronics
************

Electronics
===========

* Logic board version 1 and 2: `FPGA1394 <https://github.com/jhu-cisst/FPGA1394>`_
* Logic board version 3: `FPGA1394V3 <https://github.com/jhu-cisst/FPGA1394V3>`_

* Classic

  * Motor power board: `QLA <https://github.com/jhu-cisst/QLA>`_
  * `QLA test board <https://github.com/jhu-cisst/FPGA1394-QLA-Test>`_
  * `DQLA <https://github.com/jhu-dvrk/dvrk-DQLA>`_
  * `dMIB <https://github.com/jhu-dvrk/dvrk-pcb-dMIB>`_
  * `dSIB <https://github.com/jhu-dvrk/dvrk-pcb-dSIB>`_
  * `Intrument detection Dallas retrofit dongle <https://github.com/jhu-dvrk/dvrk_Dallas_Dongle>`_

* Si

  * Motor power board: `dRAC <https://github.com/jhu-dvrk/drac>`_
  * `Front panel LEDs <https://github.com/jhu-dvrk/dvrk-si-front-panel-led>`_
  * `ESPM programmer <https://github.com/jhu-dvrk/espm-programmer-pcba>`_
  * `dSIB <https://github.com/jhu-dvrk/dSIB-Si-pcba>`_
  * `dESSJ <https://github.com/jhu-dvrk/dESSJ-pcba>`_

Firmware/Software
=================

* Firmware for all dVRK logic boards (aka FPGA1394): `mechatronics-firmware <https://github.com/jhu-cisst/mechatronics-firmware>`_
* Client library and tools for all FPGA1394: `mechatronics-software <https://github.com/jhu-cisst/mechatronics-software>`_
* Embedded software for Zynq PS on FPGA1394 v3: `mechatronics-embedded <https://github.com/jhu-cisst/mechatronics-embedded>`_
* `dSIB Si <https://github.com/jhu-dvrk/dSIB-Si-firmware>`_ (FPGA)
* `dESSJ Si <https://github.com/jhu-dvrk/dESSJ-firmware>`_ (Arduino)

Software
********

.. note::

   Most of the code should be downloaded using ``vcs`` (python
   vcstools) instead of ``git`` to make sure all the repositories you
   need are cloned.  ``vcs`` also allows to use a specific branch or
   tag per repository to ensure compatible versions.

cisst libraries and SAW components
==================================

  * `cisst repository <https://github.com/jhu-cisst/cisst>`_
  * `cisst documentation <https://github.com/jhu-cisst/cisst/wiki>`_
  * `cisst netlib <https://github.com/jhu-cisst/cisstNetlib>`_
  * `cisst-ros <https://github.com/jhu-cisst/cisst-ros>`_ ROS 1 and 2 bridge for SAW components
  * `vcs files <https://github.com/jhu-saw/vcs>`_ for cisst, SAW components and dVRK
  * `sawTextToSpeech <https://github.com/jhu-saw/sawTextToSpeech>`_
  * `sawKeyboard <https://github.com/jhu-saw/sawKeyboard>`_
  * `sawControllers <https://github.com/jhu-saw/sawControllers>`_
  * `sawRobotIO1394 <https://github.com/jhu-saw/sawRobotIO1394>`_
  * *cisst*\ /*SAW* presentations, September 2021

    * Part 1 (30 min): https://youtu.be/SSo2MPsfBlk
    * Part 2 (45 min): https://youtu.be/XD0S4GzvfM8

CRTK
====

  * `Documentation <https://crtk-robotics.readthedocs.io>`_
  * `ROS messages <https://github.com/collaborative-robotics/crtk_msgs>`_
  * `Python ROS client library <https://github.com/collaborative-robotics/crtk_python_client>`_
  * `Matlab ROS client library <https://github.com/collaborative-robotics/crtk_matlab_client>`_

dVRK
====

  * Main site: `dVRK <https://dvrk.lcsr.jhu.edu>`_
  * Documentation `dvrk.readthedocs.io <https://dvrk.readthedocs.io>`_
  * `YouTube channel <https://www.youtube.com/channel/UCxZyIKTjk2coKGZslIOfblw>`_
  * Core components: `sawIntuitiveResearchKit <https://github.com/jhu-dvrk/sawIntuitiveResearchKit>`_
  * Python ROS client library and examples: `dvrk_python <https://github.com/jhu-dvrk/dvrk_python>`_
  * Matlab ROS client library and examples: `dvrk_matlab <https://github.com/jhu-dvrk/dvrk_matlab>`_
  * ROS URDF and launch files: `dvrk_model <https://github.com/jhu-dvrk/dvrk_model>`_
  * ROS launch files for video: `dvrk_video <https://github.com/jhu-dvrk/dvrk_video>`_
  * Configuration files from JHU: `dvrk_config_jhu <https://github.com/dvrk-config/dvrk_config_jhu>`_
  * Gravity compensation for MTMs: `dvrk-gravity-compensation <https://github.com/jhu-dvrk/dvrk-gravity-compensation>`_
  * Documentation source: `dvrk-readthedocs <https://github.com/jhu-dvrk/dvrk-readthedocs>`_
  * Continuous integration: `dvrk-github-workflow <https://github.com/jhu-dvrk/dvrk-github-workflow>`_


CAD
***

  * Files used for ROS ``dvrk_model``: `dvrk_cad <https://github.com/jhu-dvrk/dvrk_cad>`_
  * High resolution instrument models: `instrument-cad <https://github.com/jhu-dvrk/instrument-cad>`_
  * Small parts used for calibration: `dvrk-calibration-parts <https://github.com/jhu-dvrk/dvrk-calibration-parts>`_ (Classic)
  * Cannula replacement: `dvrk-cannulas <https://github.com/jhu-dvrk/dvrk-cannulas>`_ (Classic)
