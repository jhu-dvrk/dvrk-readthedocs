.. _acronyms:

********
Acronyms
********

Intuitive Surgical
##################

**ISI**
  Intuitive Surgical Inc., now known simply as Intuitive (https://wwww.intuitive.com)

**Classic**
  First generation da Vinci. The original da Vinci research kit is based on the Classic. Also known as Standard.  Patient cart came with either 2 or 3 PSMs.

**Standard**
  See above.

**S**
  Second generation da Vinci. New patient cart (PSMs, ECM, SUJs...), but surgeon console (MTMs, pedals, display...) are similar to first generation.

**Si**
  Third generation da Vinci. New surgeon console (MTMs, dual console, display, simulator...). Patient cart is similar to second generation. Supported by the dRAC based dVRK controllers.

**Si-E**
  Same as Si but with only two PSMs on the patient cart (PSM3 not included).

**Xi**
  Fourth generation da Vinci.  New setup joints, new arm to replace both PSMs and ECM (now known as USMs) can be used to hold tools or camera.  Surgeon console and stereo display similar to Si.

**X**
  Similar to Si but with Xi patient side arms.

**5**
  Latest generation, new surgeon console and patient cart based on Xi systems.  The da Vinci 5 also introduce instruments with force sensing to provide haptic feedback to the operator.

**PSM**
  Patient Side Manipulator, 2 to 3 on a full da Vinci system. Mechanically identical, they're called PSM1, PSM2, PSM3.

**MTM**
  Master Tool Manipulator, 2 on a full da Vinci system (4 with dual console on Si/Xi system). Not mechanically identical, last joints are different for left and right arms, they're called MTML and MTMR.

**ECM**
  Endoscopic Camera Manipulator, 1 on real da Vinci system.

**SUJ**
  Setup Joints, patient cart with 4 passive arms to hold active
  ECM and PSMs.  Some early versions (Standard, S, Si) came with only
  three passive arms (missing PSM3)

**HRSV**:
  High Resolution Stereo Viewer.  At least 3 versions exist, 640x480 CRT (Standard and S), 1024x768 CRT (S HD), LCD (Si/Xi/5). The Research Kit initially came with the CRT 640x480, recent sites should receive the console from S HD systems.

**CCU**
  Camera Control Unit. The two boxes in the vision cart that are connected to the endoscope cameras.  These usually have either an NTSC or SDI output for frame grabbers.

**Tray**
  Foot pedal tray: foot pedals including **clutch**, **camera**, **camera focus**, **bi** and **mono** (or **coag**).


dVRK
####

General
*******

**dVRK**
  da Vinci Research Kit

**JHU**
  `Johns Hopkins University <https://www.jhu.edu>`_

**LCSR**
  `Laboratory for Computational Sensing and Robotics <https://lcsr.jhu.edu/>`_ (at Johns Hopkins)

**cisst**
  Computer-Integrated Surgical Systems and Technology: `NSF ERC CISST <https://cisst.org>`_ and `cisst libraries <https://github.com/jhu-cisst>`_

**SAW**
  Surgical Workstation Assistant: `components based on cisst libraries <https://github.com/jhu-cisst/cisst/wiki/cisst-libraries-and-SAW-components>`_

**CRTK**
  `Collaborative Robotics ToolKit <https://crtk-robotics.readthedocs.io>`_

**ROS**
  `Robotic Operating System <https://www.ros.org>`_

dVRK specific
*************

All boards are designed by JHU and built specifically for the dVRK unless mentioned otherwise.

**QLA**
  Motor power board, Quad Linear Amplifier, JHU designed board with power for up to 4 axes.  Used in Classic arm controllers (2 per controller) and Classic SUJ controller.

**dRAC**
  Motor power board 10 PWM power lines, JHU designed board to power up to 7 motors and 3 brakes.  Used in Si controllers.

**FPGA**
  Logic board designed by JHU, mated with QLAs or dRAC. Provides 2 FireWire adapters to daisy-chain and connect to PC. Versions 2+ also includes an Ethernet adapter.  Version 3+ uses a newer FPGA, 2 FireWire and 2 Ethernet adapters, a dual-core ARM processor boots from a micro-SD card.

**QLA-FPGA**
  Board set including a QLA and FPGA board.  Used in the Classic arm controllers, up to CA9 (see :ref:`controller versions <controller-versions>`).

**DQLA**
  Dual QLA setup with a single FPGA board (version 3+).  Used in the Classic arm controllers after CA9.

**Firmware**
  In most cases, refers to embedded software running on the FPGA logic board.  For Si systems can also refer to the ESSJ or ESPM boards.

**dMIB**
  da Vinci Manipulator Interface Board, board designed to interface between the ISI manipulators using an ITT Cannon plus a foot pedal connector and the QLA-FPGA connectors (SCSI and RS cables).  Used in Classic arm controllers.

**dSIB**
  da Vinci Setup joints Interface Board.  Used in Classic SUJ controllers.

**ESPM**
  Board manufactured by ISI, embedded in S/Si PSMs and ECMs. The dVRK uses the ESPM but with a custom firmware.

**ESPM programmer**
  Board used to boot a Si PSM or ECM with a custom dVRK firmware.  The firmware is stored on a micro-SD card.

**dSIB-Si**
  Adapter board used to connect the dVRK-Si controller to a single arm and its setup joints at the base of the patient cart (either PSM or ECM)

**LVDS**
  Low-voltage differential signaling (LVDS) is a signaling
  method used for high-speed transmission of binary data over copper.
  This is used to communicate between the ESPM, ESSJ and dVRK-Si
  controller.
