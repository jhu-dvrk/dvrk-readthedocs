Introduction
############

Each patient-side da Vinci Si arm (PSM, ECM) is controlled by a single
box.  There is no dVRK support for the Si/Xi surgeon's console
(i.e. MTMs, foot pedals...).

Each controller is built around a single dRAC/FPGA stack.  Both boards
(dRAC and FPGA logic board) were designed at JHU.  The dRAC board
provided 10 PWM power lines for the PSMs and ECMs (7 motors and 3
brakes).

One main difference between the QLA and the dRAC is that the dRAC
doesn't need to process any low power signals since the embedded ESPM
does.  The ESPM converts all the low power signals (encoder,
potentiometer, buttons, instrument Id, LEDs) and communicate with the
dVRK controllers over a serial digital connection (LVDS).  To
communicate with the arm, the dVRK Si controllers use two standard
D-Sub connectors, one for the LVDS communication and one for the 10
power lines.  The connectors are the same used in the PSMs and ECMs.

The dRAC also integrates the safety chain to further reduce the number
of components.  On the Classic controllers we rely on a separate
safety relay.

All controllers come with FireWire and Ethernet interfaces so they can
be daisy chained and communicate with a computer.

All Si controllers use a single FPGA board V3.  See also
:ref:`controller versions <controller-version>`.
