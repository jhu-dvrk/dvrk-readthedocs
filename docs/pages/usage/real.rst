.. _usage-real:

**********
Real robot
**********

This section describes a typical sequence to use the dVRK. It assumes that your
dVRK is fully setup, i.e. controllers are connected, software is compiled and
all the configuration files have been generated and calibrated.

For first time users, we also have some :ref:`tips<usage-tips>`.

Powering on controllers
#######################

First thing first, you must make sure all your dVRK controllers are powered
using the black switch [0/1] next to the power cord (110/220V). We recommend
using a single power strip reserved for all your dVRK controllers. This way you
can leave all the switches to ON/1 and easily turn on all your controllers
simultaneously.

.. note::

   The dVRK controllers are quite stable and rarely need to be rebooted. At
   Johns Hopkins we keep them powered 24/7.

.. warning::

   The very first dVRK groups (circa 2012) have FPGA boards that can draw power
   from the FireWire cables. Unfortunately the power from the PC over FireWire
   is not enough to power multiple FPGA boards.  For these groups, unplug the
   FireWire cable to the PC before powering the controllers.

Once the controllers are powered, you need to make sure they booted properly. On
Classic controllers, the LEDs **A** and **B** (and **C** and **D**) should be going back and forth
from green to red. On Si controllers the **PL** LED should be blinking green.
The booting process takes a bit longer on newer controllers (FPGA 3 based). Once
all your controllers are ready, you can check the connection with your PC.

See also:

* :ref:`Classic controllers<controller-classic-exterior>` power switch and LEDs
* :ref:`Si controllers<controller-si-exterior>` power switch and LEDs

Checking connection to controllers
##################################

The best way to check the connection between the controllers and your PC is to
use ``qladisp``. If your controllers are connected using FireWire only, use:

.. code-block: bash

   qladisp -pfw

If you are using a network connection to the first controller and then FireWire
between the controllers, use:

.. code-block: bash

   qladisp -pudpfw


.. hint::

   If ``qladisp`` is not found, remember to source your ROS workspace. Something
   like ``source ~/ros2_ws/install/setup.bash``.

.. note:

   For users with FireWire only, there is a known issue related to the discovery
   of FireWire nodes after you power your controllers. This fix is to unplug the
   FireWire cable between the PC and the first dVRK controller on your chain.
   You can unplug either at the PC's end or on the controller. You won't need to
   repeat this step as long as the controllers remain powered.

The output of ``qladisp`` will provide the list of FPGA boards connected, their
board Id as well as firmware version. It also shows the type of motor power
board used (QLA1, DQLA, dRAC). ``qladisp`` is a bit verbose so, you might have to
scroll up to see the list of boards found.

If the boards are not found, please check the sections for testing connectivity
for :ref:`Ethernet<ethernet>` or :ref:`FireWire<firewire>`. You won't be able to
use the dVRK until you figure out the connection issue(s).

.. hint::

   There is a know issue regarding FireWire discovery. After you power your
   controllers, you might need to unplug, wait 5 seconds and, re-plug the
   FireWire cable between your PC and the first dVRK controller. You can
   disconnect that cable from the PC or from the first controller.

See also:

* :ref:`Connectivity<connectivity>` section
* :ref:`qladisp<qladisp>` application

Use your configuration files
############################

Make sure you use the correct configuration files! On the Classic arms, there is
no way to query the arm's serial number. Nothing prevents you from using the
configuration files for another system or another site. Si PSMs and ECMs store
their serial number on the ESPM, the dVRK software will ensure that you're using
the correct configuration files. 

.. warning::

   Using the incorrect configuration files will lead to lower
   accuracy and potentially damage your dVRK.

See also:

* :ref:`Configuration<configuration>` section

Testing motor power
###################

Once you started the :ref:`dVRK system application<system>`, try to turn on
motor power on the controllers.



.. figure:: /images/gui/gui-power-on-button.*
   :width: 300
   :align: center

   Power On button in dVRK GUI

If everything goes well, the graphical user interface should display green
buttons in the IO tabs (except for the MTM gripper since it is not powered). The
LEDs on the front of the dVRK controllers should also indicate that the motor
power is on.

.. note::

   "Turning power on" can be a bit confusing. The controllers need to be
   connected to a 110/220V and physically turned on using the switch on the
   back. At that point the FPGA (logic board) is working and communicating with
   the PC. Then, from the PC, one can send commands to turn on the motor
   amplifiers (on QLA or dRAC boards). In this document, we try to specify
   **motor** power as much as possible.


If you can not turn on motor power, 

todo
####

Things to monitor, IO frequency, temperature (htop)

Positioning for SUJ (see graph from PSM classic)

PSM sterile adapter and cannulas, Classic and Si

ECM prep for each endoscope

Tips for better accuracy

Teleoperation logic
