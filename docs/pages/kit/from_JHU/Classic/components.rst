Components
##########

Custom boards (PCBs)
********************

* :ref:`Component versions <controller-versions>` by build/date.
* The FPGA and QLA designs are open source and available on GitHub:
  https://jhu-cisst.github.io/mechatronics.
* For the DQLA based controller, the DQLA-Q and DQLA-F designs are
  open source and available on GitHub:
  https://github.com/jhu-dvrk/dvrk-DQLA
* The dMIB is provided by Intuitive Surgical. The design, including
  schematics and BOM, are available on GitHub:
  https://github.com/jhu-dvrk/dvrk-pcb-dMIB

Power supplies
**************

* All boxes contain a 12V (50W) logic power supply that provides power
  to the FPGA boards and the safety chain.
* Each box also contains one or more motor power supplies that are
  connected to the QLAs:

  * MTM: one 24V (75W) power supply connected to QLA #1 and one 12V
    (50W) power supply connected to QLA #2
  * PSM: one 24V (225W) power supply connected to both QLAs
  * ECM: one 36V (225W) power supply connected to both QLAs

* Replacement power supplies:

  * 12V Logic Power Supply (For All) & 12V Motor Power Supply (For
    MTM):
    https://www.digikey.com/product-detail/en/cui-inc/VGS-50-12/102-1935-ND/2045666
  * 24V Motor Power Supply (For MTM):
    https://www.digikey.com/product-detail/en/cui-inc/VGS-75-24/102-1943-ND/2045674
  * 24V Motor Power Supply (For PSM):
    https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-24U
  * 36V Motor Power Supply (For ECM):
    https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-36U

Hardware modifications
**********************

* dMIB:

  * :ref:`ECM switch for SUJ <dmib-ecm-pre-2015>`
  * :ref:`PSM Dallas chip for tool detection <dallas>`

* QLAs:

   * :ref:`Heat sink and fan <qla-heat-sink>`

