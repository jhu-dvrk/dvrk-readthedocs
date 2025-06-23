.. _dmib-io:

dMIB IOs reference
##################

Controllers
***********

In the table below:

* **Board**: which FPGA/QLA board set
* **I/O**: whether Input (I) or Output (O), followed by bit number
* **QLA**: signal name on QLA schematics
* **MTM**: use of this I/O by MTM controller
* **PSM**: use of this I/O by PSM controller
* **ECM**: use of this I/O by ECM controller
* **SUJ/dSIB**: use of this I/O for setup joint controller
* **dMIB pin**: which connector (DOF1-DOF8 or FP) and pin number
  (after :). Note that DOF1-DOF7 are HD15 connectors, DOF8 is a HD26
  connector, and FP (foot pedal) is a DB15 connector.

.. note::

   For Classic controllers using a DQLA/FPGA V3+, the second board Id
   is not used.  You need to add 16 to all **I/O** bit numbers for the
   second board and use the first board.  For example, "2 I:4" will
   become "1 I:20".

.. csv-table:: dMIB IOs
   :name: dmib-ios-table
   :header: "Board", "I/O", "QLA", "MTM", "PSM", "ECM", "SUJ/dSIB", "dMIB pin"
   :align: center

   "1", "I:0", "HOME1", "*daVinci Head*", "SUJ Clutch", "Manip Clutch", "Clutch 1", "DOF1:7"
   "1", "I:1", "HOME2", "", "", "", "Clutch 2", "DOF2:7"
   "1", "I:2", "HOME3", "", "Manip clutch", "SUJ Clutch", "Clutch 3", "DOF3:7"
   "1", "I:3", "HOME4", "*dVRK head*", "", "", "Clutch 4", "DOF4:7"
   "1", "I:4", "POSLIM1", "*daVinci Head*", "", "", "MotorUp", "DOF1:3"
   "1", "I:5", "POSLIM2", "", "", "", "MotorDown", "DOF2:3"
   "1", "I:6", "POSLIM3", "", "", "", "", "DOF3:3"
   "1", "I:7", "POSLIM4", "", "", "", "", "DOF4:3"
   "1", "I:8", "NEGLIM1", "*daVinci Head*", "", "", "", "DOF1:5"
   "1", "I:9", "NEGLIM2", "", "", "", "", "DOF2:5"
   "1", "I:10", "NEGLIM3", "", "", "", "", "DOF3:5"
   "1", "I:11", "NEGLIM4", "", "", "", "", "DOF4:5"
   "1", "O:0", "DOUT1", "*Endoscope* or *daVinci Head*", "", "", "NoMuxReset", "DOF1:14"
   "1", "O:1", "DOUT2", "*Endoscope*", "", "", "MuxIncrement", "DOF2:14"
   "1", "O:2", "DOUT3", "", "", "", "ControlPWM", "DOF3:14"
   "1", "O:3", "DOUT4", "", "", "", "DisablePWM", "DOF4:14"
   "2", "I:0", "HOME1", "*Foot pedal*", "", "", "", "DOF5:7, FP:5"
   "2", "I:1", "HOME2", "*Foot pedal*", "", "", "", "DOF6:7, FP:12"
   "2", "I:2", "HOME3", "*Foot pedal*", "", "", "", "DOF7:7, FP:10"
   "2", "I:3", "HOME4", "*Foot pedal*", "", "", "", "FP:1"
   "2", "I:4", "POSLIM1", "*Foot pedal*", "", "", "", "DOF5:3, FP:6"
   "2", "I:5", "POSLIM2", "*Foot pedal*", "", "", "", "DOF6:3, FP:4"
   "2", "I:6", "POSLIM3", "", "*Dallas-In*", "", "", "DOF7:3"
   "2", "I:7", "POSLIM4", "", "Tool", "", "", ""
   "2", "I:8", "NEGLIM1", "", "", "", "", "DOF5:5"
   "2", "I:9", "NEGLIM2", "", "", "", "", "DOF6:5"
   "2", "I:10", "NEGLIM3", "", "Adaptor", "", "", "DOF7:5"
   "2", "I:11", "NEGLIM4", "Arm", "Arm", "Arm", "", ""
   "2", "O:0", "DOUT1", "", "", "", "", "DOF5:14"
   "2", "O:1", "DOUT2", "", "", "", "", "DOF6:14"
   "2", "O:2", "DOUT3", "", "*Dallas-Out*", "", "", "DOF7:14"
   "2", "O:3", "DOUT4", "HE Select", "", "", "", "DOF8:16"

Explanation of signals:

* Adapter indicates whether the sterile adapter is present
* Tool indicates whether a tool (instrument) is present
* Arm is an "arm present" signal, which indicates that the arm (MTM,
  PSM or ECM) is connected
* HE Select can be used to select which Hall effect sensor (of the two
  in the MTM gripper) is read by the system. For this to be enabled,
  there must be a jumper between pins 1 and 2 of J12 on the dMIB.
* *daVinci Head* is an interface to the head-in sensor integrated in
  the full da Vinci console (see below). It is not typically present
  in dVRK systems. The above pinout assumes it is connected to DOF1 on
  the back of the controller.
* *dVRK head* is a custom head-in sensor developed for the dVRK (see
  below). The above pinout assumes it is connected to DOF4 on the back
  of the controller.
* *Endoscope* enables software control of the endoscope focus (see
  below). The above pinout assumes it is connected to both DOF1 and
  DOF2 on the back of the controller.
* *Dallas-In* and *Dallas-Out* are proposed as the interface to the
  Dallas PROM chip inside instruments, which would enable reading of
  the instrument name.

External devices
****************

Foot pedals
===========

See :ref:`da Vinci original foot pedals <config-system-console>`.

.. csv-table:: Foot pedals IOs
   :name: foot-pedals-ios-table
   :header: "Board", "I/O", "Function"
   :align: center

   "2", "I:0", "Clutch"
   "2", "I:1", "Cam-"
   "2", "I:2", "Cam+"
   "2", "I:3", "Coag"
   "2", "I:4", "Camera"
   "2", "I:5", "BiCoag"

da Vinci head sensor
====================

See :ref:`da Vinci original head sensor <config-system-console>`.

.. csv-table:: da Vinci head sensor IOs
   :name: davinci-head-sensor-ios-table
   :header: "Board", "I/O", "Function"
   :align: center

   "1", "I:0", "HeadSensor1"
   "1", "I:4", "HeadSensor2"
   "1", "I:8", "HeadSensor3"
   "1", "I:12", "HeadSensor4"
   "1", "O:0", "HeadSensorTurnOff"

dVRK head sensor
================

See :ref:`dVRK head sensor <config-system-console>`.

.. csv-table:: dVRK head sensor IOs
   :name: dvrk-head-sensor-ios-table
   :header: "Board", "I/O", "Function"
   :align: center

   "1", "I:3", "Head"

Endoscope focus controller
==========================

See :ref:`da Vinci focus controller <config-focus>`.

.. csv-table:: da Vinci focus controller IOs
   :name: davinci-focus-controller-ios-table
   :header: "Board", "I/O", "Function"
   :align: center

   "1", "O:0", "EndoscopeFocusIn"
   "1", "O:1", "EndoscopeFocusOut"


Labels on dVRK Classic controllers
==================================

The board numbering on the software side starts with index 0 (e.g. in
XML IO configuration file) but the labels on the back of the
controller start at index 1.  Keep this in mind if you plan to connect
the head sensor to another "DOF" and create your own XML configuration
files.
