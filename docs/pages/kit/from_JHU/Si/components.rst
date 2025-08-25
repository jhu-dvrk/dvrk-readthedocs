.. include:: /includes/logic-view-Si-controller.rst

Components
##########

Custom boards (PCBs)
********************

* :ref:`Component versions <controller-versions>` by build/date.
* The FPGA design is open source and available on GitHub:
* The dRAC design is open source and available on GitHub:
  https://github.com/jhu-dvrk/drac
* The LED board design is open source and available on
  GitHub: https://github.com/jhu-dvrk/dvrk-si-front-panel-led

Power supplies
**************

* All boxes contain a 12V (50W) logic power supply that provides power
  to the FPGA board and the safety chain.
* Each box also contains one motor power supply (48V, 300W) connected to the
  dRAC.

* Replacement power supplies:

  * 12V logic power supply: https://www.digikey.com/en/products/detail/mean-well-usa-inc/LRS-50-12/7705044
  * 48V motor power supply: https://www.digikey.co.nz/en/products/detail/cui-inc/VOF-300-48

Bill of materials and assembly
******************************

This information is stored in a separate GitHub repository:
https://github.com/jhu-dvrk/dVRK-Si-Controller

There is also a test board designed for the dVRK Si controllers:
* `PCB <https://github.com/jhu-dvrk/dvrk-si-test-board>`_
* `Test program <https://github.com/jhu-dvrk/dvrk-mfg-test/>`_ 
