.. _acronyms:

********
Acronyms
********

Intuitive Surgical
##################

**ISI**
  Intuitive Surgical Inc.

**Classic**
  First generation da Vinci. The original da Vinci research kit is based on the Classic. Also know as Standard.

**Standard**
  See above.

**S**
  Second generation da Vinci. New patient cart (PSMs, ECM, SUJs...), but surgeon console (MTMs, pedals, display...) are similar to first generation.

**Si**
  Third generation da Vinci. New surgeon console (MTMs, dual console, display, simulator...). Patient cart is similar to second generation. Supported by the dRAC based dVRK controllers.

**Xi**
  Fourth generation da Vinci.  New setup joints, new arm to replace both PSMs and ECM (now known as USMs) can be used to hold tools or camera.  Surgeon console and stereo display similar to Si.

**X**
  Similar to Si but with Xi patient side arms.

**PSM**
  Patient Side Manipulator, 2 to 3 on a full da Vinci system. Mechanically identical, they're called PSM1, PSM2, PSM3.

**MTM**
  Master Tool Manipulator, 2 on a full da Vinci system (4 with dual console on Si/Xi system). Not mechanically identical, last joints are different for left and right arms, they're called MTML and MTMR.

**ECM**
  Endoscopic Camera Manipulator, 1 on real da Vinci system.

**SUJ**
See Ikegami HD section, most of the system is similar except the cameras.  Some systems come with the Panasonic Cameras.  The video output is SDI.

### Cameras

![HD endoscope with Panasonic Cameras](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/video/video-panasonic-hd-endoscope.jpg)

### CCU front

![HD Panasonic CCUs front](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/video/video-panasonic-hd-ccu-front.jpg)

### CCU back

![HD Panasonic CCUs back](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/video/video-panasonic-hd-ccu-back.jpg)

  Setup Joints, patient cart with 4 passive arm to hold active ECM and PSMs.  Some early versions (Standard, S, Si) came with only three passive arms (missing PSM3)

**HRSV**:
  High Resolution Stereo Viewer.  At least 3 versions exist, 640x480 CRT (Standard and S), 1024x768 CRT (S HD), LCD (Si/Xi/5).  The Research Kit initially came with the CRT 640x480, recent sites should receive the console from S HD systems.

**CCU**
  Camera Control Unit.   The two boxes in the vision cart that are connected to the endoscope cameras.  These usually have either an NTSC or SDI output for frame grabbers.

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
  Motor power board, Quad Linear Amplifier, JHU designed board with power for up to 4 axis.  Used in Classic arm controllers (2 per controller) and Classic SUJ controller.

**dRAC**
  Motor power board 10 PWM power lines, JHU designed board to power up to 7 motors and 3 brakes.  Used in Si controllers.

**FPGA**
  Logic board designed by JHU, mated with QLAs or dRAC. Provides 2 FireWire adapters to daisy chain and connect to PC. Versions 2+ also includes an Ethernet adapter.  Version 3+ uses a newer FPGA, 2 FireWire and 2 Ethernet adapters, a dual-core ARM processor boots from a micro-SD card.

**QLA-FPGA**
  Board set including a QLA and FPGA board.  Used in the Classic arm controllers, up to CA9 (todo: see [controller versions](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions)).

**DQLA**
  Dual QLA setup with a single FPGA board (version 3+).  Used in the Classic arm controllers after CA9.

**Firmware**
  In most cases, referes to embedded software running on the FPGA logic board.  For Si systems can also refer to the dESSJ or ESPM board.

**dMIB**
  da Vinci Manipulator Interface Board, board designed to interface between the ISI manipulators using an ITT Cannon plus a foot pedal connector and the QLA-FPGA connectors (SCSI and RS cables).  Used in Classic arm controllers.

**dSIB**
  da Vinci Setup joints Interface Board.  Used in Classic SUJ controllers.

**ESPM**
  Board manufactured by ISI, embedded in S/Si PSMs and ECMs. The dVRK uses the ESPM but with a custom firmware.

**ESPM programmer**
  Board used to boot a Si PSM or ECM with a custom dVRK firmware.  The firmware is stored on a micro-SD card.

**dSIB-Si**
  Adapter board used to connect the dVRK Si controller to a single arm and its setup joints at the base of the patient cart (either PSM or ECM)

**dESSJ**
  dVRK specific board that replaces the original **ESSJ** on each setup joint of a S/Si patient cart.  The board is a pass-through for the FireWire signal and uses a BlueTooth Arduino to get (A2D) and send the SUJ joint values to the PC.

.. _davinci-generations:

Generations of da Vinci systems
###############################

Models
******

.. csv-table:: da Vinci Generations
   :name: da-vinci-generations
   :header: "Model", "Year", "Surgeon's console", "PSM/ECM/USM", "Setup Joints", "Endoscope"
   :align: center

   "Classic ", "2000", "ver 1 (640x480)", "ver 1 (PSM/ECM)", "ver 1", "ver 1 with SD"
   "S       ", "2006", "ver 1 (640x480 or 1024x768)", "ver 2 (PSM/ECM)", "ver 2", "ver 1 with SD or HD"
   "Si      ", "2009", "ver 2 HD", "ver 3 (PSM/ECM)", "ver 2", "ver 2 HD"
   "X       ", "2017", "ver 2 HD", "ver 4 (USM)    ", "ver 2", "ver 3 HD"
   "Xi      ", "2014", "ver 2 HD", "ver 4 (USM)    ", "ver 3", "ver 3 HD"
   "5       ", "2024", "ver 3 ", "ver 4 (USM)    ", "ver 3", "?"

dVRK support
************

Supported:

* Classic and S MTMs (ver1) with QLA based arm controllers
* Classic PSMs and ECMs (ver1) with QLA based arm controllers
* Classic SUJ (ver1) with QLA based SUJ controller
* Si PSMs and ECMs (ver3) with dRAC based arm controllers
* S and Si SUJ (ver2) with dESSJ and dRAC based arm controllers

Not supported:

* S PSMs and ECMs (ver2).  Some S came with ver3 PSMs or ECMs which are supported
* Si MTMs (ver 2)
* Anything X, Xi or 5
