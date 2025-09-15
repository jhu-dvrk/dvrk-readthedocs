.. _qla:

.. include:: /includes/logic-view-qla.rst

***************
Power board QLA
***************

The QLA board is designed to support up to 4 axis.  For each axis, it provides:

  * Motor power using linear amplifiers, either current or voltage mode.  Voltage
    mode was introduced in letter revisions.  The QLA supported up to 48V, 6A
  * Analog input to measure current or voltage feedback from the motor  
  * Encoder input for position feedback
  * Analog input for extra sensors such as potentiometers or Hall effect sensors
  * Limit switch inputs for homing and endstops (4)
  * Digital I/O for general purpose use

The QLA has the following connectors

  * 1 x SCSI 68 for all low voltage signals coming from or going to the robot
  * 1 x DB9 connector for all motor power connections
  * 1 x 4-pin connector for motor power input (up to 48V)
  * 3 x 2-pin connectors for fans (12V)

The QLA board also includes safety relays which can be integrated into the
systems's estop chain. To cool the 4 linear amplifiers, the board uses a large
aluminum heatsink. Active cooling was added in later revisions.

QLA boards are designed to be matted with FPGA1394 boards. The FPGA1394 board
provides some logic, including safety checks as well as the
FireWire and Ethernet interfaces to the host computer.

.. figure:: /images/fpga1394/fpga-V2-QLA-controller.jpeg
   :align: center
   :width: 400
 
   QLA board under a FPGA1394

.. note::

   The QLA board is not specifically designed for the dVRK.  It has been used in
   many different applications at JHU.

Each dVRK Classic arm controller relies on two QLA boards.  All four axis of the
first board are used for motor control.  For the PSMs and MTMs, the first 3 axis
on the second board are used.  For the ECM Classic, the first 3 axis on the
second board are used to drive the brakes. A single QLA board is also used in the
dVRK Classic SUJ controller.

Starting with FPGA V3, it is possible to control 2 QLA boards using a single
FPGA1394 logic board since the new FPGAs used have more IOs. A set of small
boards (DQLA) and flat ribbon cables are used to connect the 2 QLAs to a single
FPGA1394 board.

.. figure:: /images/fpga1394/fpga-V3-DQLA-overview.jpeg
   :align: center
   :width: 400
 
   DQLA boards and flat ribbon cables between two QLAs and one FPGA1394 V3

Links:

   * `QLA GitHub repository <https://github.com/jhu-cisst/QLA>`_
   * `QLA test board GitHub repository <https://github.com/jhu-cisst/FPGA1394-QLA-Test>`_
   * `DQLA GitHub repository <https://github.com/jhu-dvrk/dvrk-DQLA>`_