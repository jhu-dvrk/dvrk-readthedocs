.. _qla:

.. include:: /includes/logic-view-qla.rst

***************
Power board QLA
***************

The QLA board is designed to support up to 4 axis.  For each axis, it provides:

  * Motor power using linear amplifiers, either current or voltage mode.  Voltage
    mode was introduced in QLA Rev 1.5.  The QLA supports up to 48V, 6.25A
  * Feedback of the measured motor current  
  * Encoder input for position/velocity feedback
  * Analog input for extra sensors such as potentiometers or Hall effect sensors
  * Digital inputs, nominally for homing and forward/reverse limits, but also usable as general-purpose digital inputs (12)
  * Digital outputs, which can be configured as digital inputs starting with QLA Rev 1.4 (4) 

The QLA has the following connectors:

  * 1 x SCSI 68 for all low voltage signals coming from or going to the robot
  * 1 x DB9 connector for all motor power connections
  * 1 x 4-pin connector for motor power input (up to 48V)
  * 3 x 2-pin connectors for fans (12V)

The QLA board also includes safety relays which can be integrated into the
systems's estop chain. To cool the 4 linear amplifiers, the board uses a large
aluminum heatsink. Active cooling (i.e., a fan) was added in later revisions.

QLA boards are designed to be mated with FPGA1394 boards. The FPGA1394 board
provides some logic, including safety checks as well as the
FireWire and Ethernet interfaces to the host computer.

.. figure:: /images/fpga1394/fpga-V2-QLA-controller.jpeg
   :align: center
   :width: 400
 
   QLA board under a FPGA1394

.. note::

   The QLA board is not specifically designed for the dVRK.  It has been used in
   many different applications at JHU.

Each dVRK Classic arm controller relies on two QLA boards.  All four axes of the
first board are used for motor control.  For the PSMs and MTMs, the first 3 axes
on the second board are used.  For the ECM Classic, the first 3 axes on the
second board are used to drive the brakes. A single QLA board is also used in the
dVRK Classic SUJ controller.

Starting with FPGA V3, it is possible to control 2 QLA boards using a single
FPGA1394 logic board since the new FPGAs used have more IOs. In this configuration,
the FPGA1394V3 board is mounted on a DQLA-F board, which connects to a DQLA-Q board via
flat flex cables (FFC). The DQLA-Q board is mounted across both QLAs.

.. figure:: /images/fpga1394/fpga-V3-DQLA-overview.jpeg
   :align: center
   :width: 400
 
   DQLA boards and flat flex cables between two QLAs and one FPGA1394V3

Links:

   * `QLA GitHub repository <https://github.com/jhu-cisst/QLA>`_
   * `QLA test board GitHub repository <https://github.com/jhu-cisst/FPGA1394-QLA-Test>`_
   * `DQLA GitHub repository <https://github.com/jhu-dvrk/dvrk-DQLA>`_
