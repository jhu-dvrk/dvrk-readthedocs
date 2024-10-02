.. _calibration-classic-current:

Motor current
*************

.. note::

   When you are calibrating motor current, it might be simpler to have
   a single controller on the ESTOP chain.  If you prefer to keep all
   your controllers on the ESTOP chain, make sure all the controllers
   are also connected to the FireWire chain and you will have to use
   the ``qlacommand -c close-relays`` utility program to enable power.

Introduction
============

The FPGA/QLA controllers specify the commanded current between an
upper and a lower bound represented by an integer between 0 and 65,535
(D2A).  The actual values requested by the users are specified in
amperes.  The conversion factors (scales and offsets per actuator) are
specified in the XML config file.  The default offset is 65,535
divided by two, i.e. 32,768.

The same applies for the measured currents (A2D).  For both values,
the scales is based on the QLA components and we're only calibrating
the offsets for ``AmpsToBits`` and ``BitsToAmps`` in the IO XML file:

.. code-block:: xml

   <Drive>
     <AmpsToBits Offset="32768" Scale="5242.88"/>
     ...
   </Drive>

Unfortunately the requested current is not perfect and there is an
offset caused by the overall system, i.e. the controller and the robot
itself.  The goal of the calibration procedure is to:

* Power the board but not the actuators so we can read a zero current
  feedback
* Compute the measured current offset
* Power actuators and request a null current on all actuators knowing
  that it won't be totally null
* Measure the current feedback
* Update the requested current offsets based on the difference between
  requested and measured current
* Save the new offsets in the IO XML configuration file

Procedure
=========

The program to calibrate the requested current is called
`sawRobotIO1394CurrentCalibration`.  To use it you will need an
existing XML configuration file.  The command line options are:

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

For most users, the default port (FireWire 0) should.  If you're using
Ethernet for your controller data connection, use ``-p udp``.  By
default, should be able to run the program using something like:

.. code-block:: bash

   # use the IO XML file corresponding to the arm/controller you're calibrating
   sawRobotIO1394CurrentCalibration -c sawRobotIO1394-MTML-12345.xml

The program takes a few seconds to run and the expected output is
something like:

::

   adeguet1@lcsr-dvrk-09:~/ros2_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci$
     sawRobotIO1394CurrentCalibration -c sawRobotIO1394-PSM1-49695.xml

   Configuration file: sawRobotIO1394-PSM1-49695.xml
   Port: fw:0
   Make sure:
   - your computer is connected to the firewire controller.
   - the arm corresponding to the configuration file "sawRobotIO1394-PSM1-49695.xml" is connected to the controller.
   - the E-Stop is closed, i.e. will let the controller power on.
   - you have no other device connected to the firewire chain.
   - you have no other program trying to communicate with the controller.

   Press any key to start.
   Loading config file ...
   FirewirePort::Init: number of ports = 1
   Port 0: /dev/fw11, 12 nodes
   FirewirePort::Init: successfully initialized port 0
   Using libraw1394 version 2.1.2
   FirewirePort::Init: successfully disabled cycle start packet
   FirewirePort::InitNodes: base node id = ffc0
   BasePort::ScanNodes: building node map for 11 nodes:
     Node 0, BoardId = 12, FPGA_V2, Hardware = QLA1, Firmware Version = 8
     Node 1, BoardId = 5, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 2, BoardId = 4, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 3, BoardId = 8, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 4, BoardId = 9, FPGA_V2, Hardware = QLA1, Firmware Version = 8
     Node 5, BoardId = 6, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 6, BoardId = 7, FPGA_V2, Hardware = QLA1, Firmware Version = 8
     Node 7, BoardId = 3, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 8, BoardId = 2, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 9, BoardId = 1, FPGA_V1, Hardware = QLA1, Firmware Version = 8
     Node 10, BoardId = 0, FPGA_V1, Hardware = QLA1, Firmware Version = 8
   BasePort::ScanNodes: found 11 boards
   BasePort::SetDefaultProtocol: all nodes broadcast capable and support shorter wait
   W- Class mtsRobotIO1394: SetBoards: PSM1, board: 0, Id: 6, firmware: 8, FPGA serial: unknown, QLA serial: unknown
   W- Class mtsRobotIO1394: SetBoards: PSM1, board: 1, Id: 7, firmware: 8, FPGA serial: 4981-14, QLA serial: 4980-14

   Creating robot ...

   Ready to power?  Press any key to start.
   Enabling power to the QLA board...
   Status: power seems fine.
   Starting calibration ...
   Measured current error statistics
   Status: average current feedback in mA: 	0.328691	-0.488023  	51.6638 	0.314944 	0.498726 	0.717084	0.0938034
   Status: standard deviation in mA:        	1.20528  	1.12293  	2.42928  	1.27400  	1.02623  	1.12980  	1.20621
   Status: kept 3418 samples out of 50000
   Status: new average in mA:              	0.292594	-0.455903  	51.9898 	0.323646 	0.411650 	0.777404	0.0646454

   Enabling amplifiers for the actuators...
   Status: power seems fine.
   Starting calibration ...
   Commanded current error statistics
   Status: average current feedback in mA:	-0.406738	-0.473329 	-28.8600	-0.372567  	1.28329	-0.733715	-0.504301
   Status: standard deviation in mA:        	1.17603  	1.12665  	1.34004  	1.26102  	1.10190  	1.19607  	1.26267
   Status: kept 3024 samples out of 50000
   Status: new average in mA:             	-0.372473	-0.445304 	-28.8130	-0.318020  	1.28542	-0.738004	-0.496518

   Status: measured current offsets in mA: 	0.292594	-0.455903  	51.9898 	0.323646 	0.411650 	0.777404	0.0646454
   Status: command current offsets in mA (corrected):	-0.665067	0.0105987 	-80.8027	-0.641666 	0.873767 	-1.51541	-0.561164

   Status: commanded current offsets in XML configuration file:  	33087.0  	32650.0  	33060.0  	33035.0  	32919.0  	32971.0  	32894.0
   Status: new commanded current offsets:                        	33083.5  	32650.1  	33483.6  	33031.6  	32923.6  	32978.9  	32896.9
   Status: measured current offsets in XML configuration file:  	6.28967  	6.23475 	-6.27485  	6.27740  	6.24977 	-6.24615 	-6.25612
   Status: new measured current offsets:                        	6.28938  	6.23520 	-6.32684  	6.27707  	6.24936 	-6.24692 	-6.25619

   Do you want to update the config file with these values? [y(es)/n(o)]
   Existing IO config file has been renamed sawRobotIO1394-PSM1-49695.xml-backup-2024-10-01-14-02-34
   Results saved in IO config file sawRobotIO1394-PSM1-49695.xml

Notes:

* If the program fails and displays endless ``WriteAllBoards: handle
  for port 0 is NULL``, hit ctrl-c to stop it.  Then test with
  ``qladisp`` to make sure your data connection is good (FireWire or
  Ethernet).

* If the program fails to power the controllers, make sure you can
  power the controllers using the utility ``qladisp``.

* If you are calibrating an MTM, please keep in mind that the last
  actuator (8) is not powered so you can ignore the last column.

* The values for ``new average in mA`` shouldn't exceed more than a
  few tens of mA.  If you have significantly higher values, DO NOT
  PROCEED and reach out to the dVRK maintainers.

* Finally, it is recommended test the new offsets by re-running the
  calibration utility.  At the point, the offsets in mA should be
  close to 0 (few tenths of mA) and the offsets in the XML file should
  be close to 0:

::

   Status: new average in mA:                  0.100924   -0.0292067    -0.230016   -0.0497301     0.135562   -0.0820793     0.131824      8.34862
   Status: current offsets in XML configuration file:      32811.0      32941.0      32709.0      32859.0      32823.0      32858.0      32907.0      32634.0
   Status: new current offsets:                            32811.5      32941.2      32710.2      32859.3      32823.7      32858.4      32907.7      32590.2
