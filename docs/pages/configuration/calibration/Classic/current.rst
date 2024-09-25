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

   Configuration file: sawRobotIO1394-MTML-00000.xml
   Port: 0
   Make sure:
   - your computer is connected to the firewire controller.
   - the arm corresponding to the configuration file "sawRobotIO1394-MTML-00000.xml" is connected to the controller.
   - the E-Stop is closed, i.e. will let the controller power on.
   - you have no other device connected to the firewire chain.
   - you have no other program trying to communicate with the controller.

   Press any key to get started.
   Loading config file ...
   Creating robot ...
   Creating port ...
   FirewirePort: number of ports = 1
   FirewirePort: successfully initialized port 0
   ScanNodes: base node id = ffc0
   ScanNodes: building node map for 9 nodes:
    Node 0, BoardId = 6, Firmware Version = 3
    Node 1, BoardId = 7, Firmware Version = 3
    Node 2, BoardId = 8, Firmware Version = 3
    Node 3, BoardId = 9, Firmware Version = 3
    Node 4, BoardId = 0, Firmware Version = 3
    Node 5, BoardId = 1, Firmware Version = 3
    Node 6, BoardId = 3, Firmware Version = 3
    Node 7, BoardId = 2, Firmware Version = 3

   Ready to power?  Press any key to start.
   Enabling power ...
   Status: power seems fine.
   Starting calibration ...
   Status: average current feedback in mA:      8.35252     -33.1456      11.4888      -17.2173      10.3679     -16.9912      26.5735      8.29041
   Status: standard deviation in mA:            1.12377      1.11879      1.23513       1.13384      1.21539      1.18651      1.11418      1.17129
   Status: kept 2458 samples out of 50000
   Status: new average in mA:                   8.39934     -33.1659      11.6538      -17.1952      10.3920     -16.9736      26.6081      8.33897

   Do you want to update the config file with these values? [Y/y]
   Status: current offsets in XML configuration file:      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0      32768.0
   Status: new current offsets:                            32812.0      32941.9      32706.9      32858.2      32822.5      32857.0      32907.5      32724.3

   Do you want to save these values? [S/s]
   Status: new configuration file is "sawRobotIO1394-MTML-00000.xml-new"

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
