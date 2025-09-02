.. _calibration-classic-current:

Motor current
*************

.. note::

   When you are calibrating motor current, it might be simpler to have
   a single controller on the ESTOP chain.  If you prefer to keep all
   your controllers on the ESTOP chain, make sure all the controllers
   are also connected to the FireWire chain, and you will have to use
   the ``qlacommand -c close-relays`` :ref:`utility program
   <qlacommand>` to enable power.

Introduction
============

The FPGA/QLA controllers specify the commanded current between an
upper and a lower bound represented by an integer between 0 and 65,535
(D2A).  The actual values requested by the users are specified in
amperes.  The conversion factors (scales and offsets per actuator) are
specified in the JSON config file.  The default offset is 65,535
divided by two, i.e. 32,768.

The same applies for the measured currents (A2D).  For both values,
the scales is based on the QLA components, and we're only calibrating
the offsets for ``current_to_bits`` and ``bits_to_current`` in the IO JSON file:

.. code-block:: json

   "drive": {
     "bits_to_current": {
       "offset": 6.2787991134553245,
       "scale": -0.000190735
     },
     "current_to_bits": {
       "offset": 32937.59236287833,
       "scale": -5242.88
     }
   }

Unfortunately the requested current is not perfect and there is an
offset caused by the overall system, i.e. the controller and the robot
itself.  The goal of the calibration procedure is to:

* Power the board but not the actuators, so we can read a zero current
  feedback
* Compute the measured current offset
* Power actuators and request a null current on all actuators knowing
  that it won't be totally null
* Measure the current feedback
* Update the requested current offsets based on the difference between
  requested and measured current
* Save the new offsets in the IO JSON configuration file

Procedure
=========

The program to calibrate the requested current is called
:ref:`sawRobotIO1394CurrentCalibration
<sawrobotiocurrentcalibration>`.  To use it, you will need an existing
JSON configuration file.  The command line options are:

::

   sawRobotIO1394CurrentCalibration:
     -c <value>, --config <value> : configuration file (required)
     -p <value>, --port <value> : firewire port number(s) (optional)
     -b, --brakes : calibrate current feedback on brakes instead of actuators (optional)

.. _calibration-classic-current-brakes:

The ``--brakes`` option is used to calibrate the brakes on the ECM arm
only.  **For the ECM, the procedure needs to be executed twice, once
for the brakes (with ``-b``) and once for the actuators (without
``-b``)**.

For most users, the default port (FireWire 0, i.e. ``-p fw``) should work.  If you're using
Ethernet for all your controllers data connection, use ``-p udp``.  If you're
using Ethernet between the PC and the first dVRK controller and then FireWire
between the controllers, use ``-p udpfw``.  You should be able to run the program using something like:

.. code-block:: bash

   # use the IO JSON file corresponding to the arm/controller you're calibrating
   sawRobotIO1394CurrentCalibration -c sawRobotIO1394-MTML-12345.json

The program takes a few seconds to run, and the expected output is
something like:

::

  sawRobotIO1394CurrentCalibration -c sawRobotIO1394-MTML-22723.json
  Configuration file: sawRobotIO1394-MTML-22723.json
  Port: fw:0
  Make sure:
   - your computer is connected to the firewire controller.
   - the arm corresponding to the configuration file "sawRobotIO1394-MTML-22723.json" is connected to the controller.
   - the E-Stop is closed, i.e. will let the controller power on.
   - you have no other device connected to the firewire chain.
   - you have no other program trying to communicate with the controller.

  Press any key to start.
  Loading config file...
  FirewirePort::Init: number of ports = 1
    Port 0: /dev/fw8, 9 nodes
  FirewirePort::Init: successfully initialized port 0
  Using libraw1394 version 2.1.2
  FirewirePort::Init: successfully disabled cycle start packet
  FirewirePort::InitNodes: base node id = ffc0
  BasePort::ScanNodes: building node map for 8 nodes:
    Node 0, BoardId = 8, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 1, BoardId = 9, FPGA_V2, Hardware = QLA1, Firmware Version = 8
    Node 2, BoardId = 6, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 3, BoardId = 7, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 4, BoardId = 0, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 5, BoardId = 1, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 6, BoardId = 2, FPGA_V1, Hardware = QLA1, Firmware Version = 8
    Node 7, BoardId = 3, FPGA_V1, Hardware = QLA1, Firmware Version = 8
  BasePort::ScanNodes: found 8 boards
  BasePort::SetDefaultProtocol: all nodes broadcast capable and support shorter wait
  Creating robot...
  > Robot created
  Enabling power to the QLA board...
  ..........
  > Power is fine
  Starting calibration...
  ..................................................
  Measured current error statistics
  Average current feedback in mA:     -28.8000      15.6491     0.434221      13.3348     -17.7697      13.4818     -4.60474
  Standard deviation in mA:            1.12378      1.09176      1.05553      1.17422      1.16107      1.13963      1.15817
  New average in mA:                  -28.7991      15.6453     0.325076      13.4372     -17.8255      13.5361     -4.62803
  Kept 3033 samples out of 50000

  Enabling amplifiers for the actuators...
  ..........
  > Power is fine
  Starting calibration...
  ..................................................
  Commanded current error statistics
  Average current feedback in mA:      3.52535     -3.78245     -20.2159     -22.7148      5.44067     -1.14394      14.9589
  Standard deviation in mA:            1.10327      1.08878      1.16337      1.15764      1.15749      1.01432      1.14087
  New average in mA:                   3.54806     -3.82199     -20.2423     -22.7453      5.45498     -1.03438      14.9746
  Kept 3923 samples out of 50000

  Measured current offsets in mA:     -28.7991      15.6453     0.325076      13.4372     -17.8255      13.5361     -4.62803
  Command current offsets in mA:       32.3472     -19.4673     -20.5674     -36.1825      23.2804     -14.5705      19.6026

  Old commanded current offsets:      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0
  New commanded current offsets:      32937.6      32870.1      32875.8      32957.7      32890.1      32844.4      32870.8
  Old measured current offsets:       6.25000     -6.25000     -6.25000     -6.25000      6.25000     -6.25000      6.25000
  New measured current offsets:       6.27880     -6.26565     -6.25033     -6.26344      6.26783     -6.26354      6.25463

  Do you want to update the config file with these values? [y(es)/n(o)]
  Existing IO config file has been renamed sawRobotIO1394-MTML-22723.json-backup-2025-08-29_12:30:48:324
  Results saved in IO config file sawRobotIO1394-MTML-22723.json

Notes:

* If the program fails and displays endless ``WriteAllBoards: handle
  for port 0 is NULL``, hit ctrl-c to stop it.  Then test with
  ``qladisp`` to make sure your data connection is good (FireWire or
  Ethernet).

* If the program fails to power the controllers, make sure you can
  power the controllers using the utility ``qladisp``.

* The values for ``new average in mA`` shouldn't exceed more than a
  few tens of mA.  If you have significantly higher values, DO NOT
  PROCEED and reach out to the dVRK maintainers.

* Finally, it is recommended test the new offsets by re-running the
  calibration utility.  At the point, the offsets in mA should be
  close to 0 (few tenths of mA) and the offsets in the JSON file should
  be close to 0
