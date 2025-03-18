************
Introduction
************

To schematize, robot control requires:

**1** Robot sensors to measure the state of the robot, including limit switches,

  **2** Digital serialization of analog signals provided by the sensors using ADC, encoder chips...

    **3** Converting the digital signals from the serialization to data usable by the control algorithm, integers or floating point numbers (for example PID)

      **4** Computing a command to be sent to the actuators based on the current state

    **5** Converting the command from integers or floating point numbers to bits based on the amplifiers used

  **6** Converting to motor commands using linear amplifiers or PWM chips (current, voltage, duty cycle...)

**7** Actuators to make the robot move

The design of the dVRK is based on centralized computing (**4**) on a PC and
distributed I/O on the controllers (**2**, **3**, **5** and **6** over FireWire
or Ethernet).

The controllers themselves use two distinct components, a logic board (also
known as FPGA or FPGA1394) for **3** and **5** and a board for signal processing
and motor/brake power for: QLA for da Vinci Classic (for **2** and **6**) and
dRAC for da Vinci Si (for **6**).  The custom electronic components (FPGA, QLA
and dRAC) are all designed by the Johns Hopkins group under the direction of Dr.
Peter Kazanzides. The board designs, bills of materials, and all the firmware
source is freely available on GitHub.

Different boards are used on different configuration and generations of dVRK
controllers. Some boards are specific to each da Vinci version and are described
in later sections: dMIB, dSIB, DQLA, dSIB-Si, dSIB-Si, dSIB-Z-Si. These boards
are mostly adapters between different types of existing connectors.

For the dVRK-Si, most of the signal processing (encoders, potentiometers...) is
performed on original boards embedded in the da Vinci arms (**2**).  Instead of
designing and manufacturing custom boards, we are able to re-use the existing
boards (ESPM and ESSJ) but with a custom firmware.

In the figure below, you can see the overall design for the dVRK Classic and Si
controllers for both the active arms (PSM, ECM and MTM) and SUJs.

.. figure:: /images/controllers/dVRK-signals-all-controllers.*
   :width: 600
   :align: center

   Overall view of dVRK controllers integration

All acronyms are defined in the :ref:`reference section <acronyms>`.  The
following sections provide an overview of the main dVRK custom boards: FPGA, QLA
and dRAC.
