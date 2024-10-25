Introduction
############

Each da Vinci arm (MTM, PSM, ECM) is controlled by a single box. A
different controller is used for the Setup joints (see :ref:`SUJ
Classic controller <constrollers-suj-classic>`).  There are three
major major versions of controllers for the da Vinci Classic, based on
the FPGA version used (V1, V2 or V3).

The V1 and V2 controllers are built around 2 QLA/FPGA stacks.  Both
boards (QLA and FPGA logic board) were designed by JHU.  Since a QLA
(Quad Linear Amplifier) can only drive up to 4 axes, we need two of
them per controller.

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
complexity of wiring inside the controller.  See also :ref:`controller
versions <controller-version>`.
