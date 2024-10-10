*******
Classic
*******

Overview
########

Each da Vinci arm (MTM, PSM, ECM) is controlled by a single box. A
different controller is used for the Setup joints (see below).  There
are three major versions of controllers, based on the FPGA version
used (V1, V2 or V3). This section documents the V1 and V2 controllers;
documentation for the V3 controllers is forthcoming.

The V1 and V2 controllers are built around 2 QLA/FPGA stacks.  Both boards (QLA and FPGA logic board) were designed by JHU.  Since a QLA (Quad Linear Amplifier) can only drive up to 4 axes, we need two of them per controller.

The controllers are designed to interface with the da Vinci Classic
(first generation) active arms, both on the patient and surgeon's
side.  The physical interface between the QLA/FPGA boards is a dMIB
(da Vinci Manipulator Interface Board).  The controllers provide
inputs for the potentiometers and encoders as well as miscellaneous
digital IOs (foot pedals, buttons...).  For motor control they use
linear amplifiers with current feedback.  All controllers come with
FireWire interfaces so they can be daisy chained and communicate with
a computer.

The V2 controllers also come with an Ethernet adapter (supported by
Firmware Rev 7+ and Software V2+).  The V3 controllers are still based
on 2 QLAs but use a single FPGA board (V3) to reduce the cost and
complexity of wiring inside the controller.  See also :ref`controller
versions <controller-version>`.

Exterior Connectors
###################

* One AC power connector, with on/off switch
* One 156-pin connector ITT Cannon (for the MTM, PSM, or ECM)
* Two FireWire connectors
* Two Ethernet connectors (V2 and V3 controllers)
* One or two 4 or 5-pin safety chain connectors (depending on
  version); see [ESTOP
  page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/ESTOP)
* One DB15 footpedal connector; see [dMIB I/O
  page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dMIB-IOs)
* Seven HD15 expansion connectors and one HD26 expansion connector;
  see [dMIB I/O page](/jhu-dvrk/sawIntuitiveResearchKit/wiki/dMIB-IOs)

Internal Components
###################

Internally, each V1 or V2 QLA1-based controller box contains two
FPGA/QLA board sets, one dMIB (da Vinci Manipulator Interface Board),
LED boards, power supplies and relays.

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/qla1-controller-diagram.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/qla1-controller-diagram.png" width="500"></a>

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/qla1-controller-layout.jpg"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/qla1-controller-layout.jpg" width="500"></a>

The V3 DQLA1-based controllers contain one FPGA V3 and 2 QLA boards, a set of DQLA-Q/DQLA-F (connected using 2 flat ribbons), one dMIB (da Vinci Manipulator Interface Board), LED boards, power supplies and relays.  The FPGA is mounted against the controller's back panel so we can use the FireWire and Ethernet ports directly.  Compared to the QLA1-based controllers, this greatly reduced the amount of internal wiring (Ethernet and Firewire pass-through cables).

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/dqla-controller-diagram.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/dqla-controller-diagram.png" width="500"></a>

<a href="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/dqla-controller-layout.png"><img src="/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/controllers/dqla-controller-layout.png" width="500"></a>

### Custom Boards (PCBs)
* [Component versions](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions) by build/date.
* The FPGA and QLA designs are open source and available on [GitHub](http://jhu-cisst.github.io/mechatronics/).
* For the DQLA based controller, the DQLA-Q and DQLA-F designs are open source and available on [GitHub](https://github.com/jhu-dvrk/dvrk-DQLA)
* The dMIB is provided by Intuitive Surgical. The designs, including schematics and BOM, are available on [GitHub](https://github.com/jhu-dvrk/dvrk-pcb-dMIB).

### Power Supplies
* All boxes contain a 12V (50W) logic power supply that provides power to the FPGA boards and the safety chain.
* Each box also contains one or more motor power supplies that are connected to the QLAs:
  * MTM: one 24V (75W) power supply connected to QLA #1 and one 12V (50W) power supply connected to QLA #2
  * PSM: one 24V (225W) power supply connected to both QLAs
  * ECM: one 36V (225W) power supply connected to both QLAs
* Replacement power supplies
  * 12V Logic Power Supply (For All) & 12V Motor Power Supply (For MTM): https://www.digikey.com/product-detail/en/cui-inc/VGS-50-12/102-1935-ND/2045666
  * 24V Motor Power Supply (For MTM): https://www.digikey.com/product-detail/en/cui-inc/VGS-75-24/102-1943-ND/2045674
  * 24V Motor Power Supply (For PSM): https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-24U
  * 36V Motor Power Supply (For ECM): https://www.astrodynetdi.com/ecatalog/power-supplies/PMK225S-36U

## Hardware modifications

* dMIB:
  * [ECM switch for SUJ](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Full-da-Vinci-dMIB-pre-2015)
  * [PSM Dallas chip for tool detection](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Tool-Detection)
* QLAs:
  * [Heat sink and fan](/jhu-dvrk/sawIntuitiveResearchKit/wiki/QLA-Heat-Sink)
