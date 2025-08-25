.. _controllers-suj-classic:

***********
Classic SUJ
***********

Introduction
############

The dVRK Classic SUJ controllers are similar to the :ref:`dVRK Classic
controllers <controllers-classic>`.  They are build around a single
QLA/FPGA stack (using a FPGA V1 or V2) and a special board to
interface with the SUJ arms, the dSIB (da Vinci SUJ Interface Board).
Each controller has 4 156-pin connectors Cannon ITT, one for each SUJ
arm.  A single dVRK SUJ controller is required per patient cart.

The dVRK SUJ controller supports all the features available on the
da Vinci Classic patient cart, i.e.:

* Read joint positions. The dVRK QLA has 4 analog to digital inputs, so
  it reads the potentiometer values sequentially using a multiplexer
  on the dSIB.
* Release brakes. The dVRK controller uses the linear amps of the QLA
  dedicated to motor control to release the brakes.
* Lift PSM3. The dVRK FPGA generates a PWM signal sent to the PWM
  power unit included on the dSIB.
* Read the brake/clutch buttons on the SUJ-PSM1 and SUJ-PSM2.  The
  brake buttons on the active arms themselves are handled by each
  active arm's controller, not the SUJ controller (see :ref:`Classic
  SUJ <suj>`)
  
See also :ref:`controller versions <controller-version>`.


Exterior
########

Connectors
**********

* One AC power connector, with on/off switch
* Four 156-pin connector ITT Cannon (for the MTM, PSM, or ECM)
* Two FireWire connectors
* One Ethernet connectors
* One 4-pin and one 5-pin safety chain connector; see :ref:`E-Stop <estop>`

.. figure:: /images/controllers/suj-controller-exterior.png
   :width: 600
   :align: center

   Classic SUJ controller


.. figure:: /images/Classic/SUJ/SUJ-Classic-controller-ITT-connectors.jpeg
   :width: 400
   :align: center

   Classic SUJ controller arm connections

LEDs
****

The LEDs are based on the Classic controllers; see :ref:`dVRK Classic
controllers <classic-leds>`


Internal layout
###############

Internally, each controller box contains one FPGA/QLA board set, one
dSIB (da Vinci SUJ Interface Board), LED boards and 2 power supplies:

* 12V (50W) logic power supply that provides power to the FPGA board
* Brake power supply connected to the QLA (48V) with built-in
  safety-relay.

.. figure:: /images/controllers/suj-controller-interior.png
   :width: 600
   :align: center

   Classic SUJ controller internals

The dSIB has been designed to plug directly into the QLA board (SCSI
and D-sub connectors).  This reduces the amount of cabling inside the
controller.

Components
##########

Custom boards (PCBs)
********************

* :ref:`Component versions <controller-versions>` by build/date.
* The FPGA and QLA designs are open source and available on GitHub:
  https://jhu-cisst.github.io/mechatronics.
* The dSIB is provided by Intuitive Surgical. The design, including
  schematics and BOM, are available on GitHub:
  https://github.com/jhu-dvrk/dvrk-pcb-dSIB

Power supplies
**************

* All boxes contain a 12V (50W) logic power supply that provides power
  to the FPGA board.
* Each box also contains one motor supply (48V) connected to the QLA:

Hardware modifications
**********************

* QLAs:

  * :ref:`Heat sink and fan <qla-heat-sink>`

Details of components and assembly
**********************************

This information is stored in a separate GitHub repository:
https://github.com/jhu-dvrk/dVRK-Classic-SUJ-Controller
