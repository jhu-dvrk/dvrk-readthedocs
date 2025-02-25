.. _firewire:

********
FireWire
********

Important notes
###############

.. caution::

    FireWire A connectors have 6 pins. Two of them carry power to
    drive an external device (e.g. hard drive). The dVRK doesn't use
    these two pins but if a user accidentally force the connector
    upside-down, the two power pins will instead touch the data lines
    and **irreversibly** damage the dVRK FPGA board. Make sure you
    respect the direction of the FireWire connector. Think of it as
    old USB connectors, these are not reversible like USB-C,
    Lightning... If you have to force it to plug it, visually check
    that the connector is not upside-down.


* There is no such thing as a FireWire to USB adapter, so you will need
  a computer with a FireWire adapter
* FireWire is not supported by virtualization software (VMWare Fusion,
  Parallel...) so you will need to use a native Linux OS (we strongly
  recommend Ubuntu LTS)
* You should never have two PCs connected to the same FireWire chain
* Avoid mixing dVRK controllers and other FireWire devices on the same
  bus (e.g. older Sensable Phantom Omni, FireWire cameras...)
* Board Id and FireWire node numbers are different ways to identify
  the FPGA boards on the FireWire bus. These numbers usually don't
  match (you can use :ref:`qladisp` without options to see the mapping
  of nodes/board Ids)
* The node number is determined by the FireWire bus (physical layer,
  or PHY) and the numbers should be sequential. Node 0 usually
  corresponds to ``/dev/fw1``, 1 to ``/dev/fw2``, etc. If you have
  multiple FireWire adapters, the numbering for ``/dev/fw*`` might
  start higher, i.e. node 0 would be associated to ``/dev/fw2``...
* The :ref:`board Id<board-id>` is determined by the physical dial on
  the FPGA board and is used to identify which controller is connected

Setup
#####

FireWire adapter
****************

You will need a FireWire adapter on the PC. Due to the fact that
FireWire is a sophisticated protocol, some chipset implementations are
not fully functional and have various issues such as dropping packets
and supporting a limited number of FireWire nodes. We recommend
adapters with chipsets from Texas Instruments with native PCIe support
(i.e. not with PCIe to PCI bridge).  See :ref:`PC
configuration<user-computers>`.

To get the chipset model of your FireWire card, use ``lshw``.

Here is an example of output for a chip known to work on PCIe bus
(recent computers):

::

   *-firewire
      description: FireWire (IEEE 1394)
      product: XIO2213A/B/XIO2221 IEEE-1394b OHCI Controller [Cheetah Express]
      vendor: Texas Instruments

And an example of output for a PCI based chip for much older computers:

::

   *-firewire
      description: FireWire (IEEE 1394)
      product: TSB12LV23 IEEE-1394 Controller
      vendor: Texas Instruments

Drivers
*******

You have to install libraw1394. This has to be done only once per
computer by a user with sudo privileges:

.. code-block:: bash

   sudo apt-get install libraw1394-dev


Set permissions for FireWire devices
************************************

In order to run the control software without root/sudo permissions,
please follow the following steps:

* Create ``/etc/udev/rules.d`` folder if it's not there
* Add rules for ``/dev/fw*`` devices
* Optionally create group ``fpgaqla``
* Optionally add the current user to group ``fpgaqla``
* Reload ``udev`` rules

Note: pick one of the two solutions described below!

Convenient solution
===================

If you or your institution doesn't care about who can access the
FireWire devices on your system, you can grant anyone to have read and
write permissions on all FireWire devices. This is simpler to manage
and should satisfy the requirements of most, if not all, dVRK users.

The following script should be run only once per computer:

.. code-block:: bash

   sudo mkdir -p /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0666"' > ~/80-firewire-all.rules # create the rule
   sudo mv ~/80-firewire-all.rules /etc/udev/rules.d/80-firewire-all.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo udevadm control --reload-rules # apply new rules

Safer solution
==============

If you or your institution *really*, *really* cares about who can access
the FireWire devices on your computer, you can create a dedicated Unix
group to control who can access the FireWire devices.

The following script should be run only once per computer and performs
the steps described above:

.. code-block:: bash

   sudo mkdir -p /etc/udev/rules.d # create a directory if needed
   cd
   echo 'KERNEL=="fw*", GROUP="fpgaqla", MODE="0660"' > ~/80-firewire-fpgaqla.rules # create the rule
   sudo mv ~/80-firewire-fpgaqla.rules /etc/udev/rules.d/80-firewire-fpgaqla.rules  # move the rule in the proper directory
   sudo addgroup fpgaqla          # create the group with read-write access to /dev/fw*
   sudo udevadm control --reload-rules # apply new rules
   sudo adduser `whoami` fpgaqla  # add current user to the group

For all additional users, you will need to add the new user to the
group. To find the user Id, one can either use the command ``id`` or do
``ls /home``. Once the user Id is known, someone with sudo privileges
should do:

.. code-block::

   sudo adduser put_the_new_user_id_here fpgaqla

Once a user has been added to the ``fpgaqla`` group, they need to
logout/login so the group membership can take effect. To check if the
group membership is correct, the user can use the shell
command ``id``. See! It's a mess, so you should really use the convenient
solution instead :-)


Testing connectivity
####################

The number of expected logic boards depends on each setup, see
:ref:`number of FPGAs <nb-fpgas>` section.

.. _firewire-qladisp:

``qladisp``
***********

Note: qladisp is part of the dVRK software, so you will have to build
the software first. See :ref:`software build
instructions<software>`.

There are a few ways to test that your controllers are properly
connected. You can start with the command line application provided
with the dVRK software qladisp. Just type qladisp in a terminal
(without options) and the output should show the list of boards found
with their board Id and firmware version. For example:

.. code-block::

   Trying to detect boards on port:
   ParseOptions: no option provided, using default fw:0
   FirewirePort::Init: number of ports = 1
     Port 0: /dev/fw12, 14 nodes
   FirewirePort::Init: successfully initialized port 0
   Using libraw1394 version 2.1.2
   FirewirePort::Init: successfully disabled cycle start packet
   FirewirePort::InitNodes: base node id = ffc0
   BasePort::ScanNodes: building node map for 13 nodes:
     Node 0, BoardId = 12, Firmware Version = 7
     Node 1, BoardId = 10, Firmware Version = 7
     Node 2, BoardId = 11, Firmware Version = 7
     Node 3, BoardId = 5, Firmware Version = 7
  ...

This is the output for a full system. For most systems, you should see
two boards per controller/arm.

``ls -l /dev/fw*``
******************

If qladisp doesn't work, check that all FireWire devices have been
found and created with the correct files permissions using ``ls -al
/dev/fw*``. The output should look like:

.. code-block::

   crw-rw-rw- 1 root fpgaqla 243,  0 Feb 12 09:31 /dev/fw0
   crw-rw-rw- 1 root fpgaqla 243,  1 Mar  2 11:45 /dev/fw1
   crw-rw-rw- 1 root fpgaqla 243,  2 Mar  2 11:45 /dev/fw2
   crw-rw-rw- 1 root fpgaqla 243,  3 Mar  2 11:45 /dev/fw3
   crw-rw-rw- 1 root fpgaqla 243,  4 Mar  2 11:45 /dev/fw4
   ...

You should have two fw devices created for each controller (except 1
for the SUJ controller). Note that fw0 is the FireWire adapter on the
PC itself. If you have multiple FireWire cards on your PC, the first
nodes will correspond to the cards on the PC (e.g. for 2 cards, fw0
and fw1).

.. warning::

   The fw devices should be numbered contiguously, i.e. there
   shouldn't be any gap between the numbers. If there are some gaps,
   the FireWire bus initialization likely failed. This can happen when
   FireWire cables are unplugged and re-plugged too fast for the
   kernel so make sure you wait a few seconds between steps. If this
   happens, you can force a bus reset by unplugging, waiting 5 seconds
   and re-plugging the FireWire cable on your PC.

.. _dmesg:

``dmesg -w``
************

You can also monitor the kernel messages using the command ``dmesg
-w``. Start the command in a separate terminal and leave it alone
while plugging/unplugging the FireWire cables. You should see messages
re. the creation of FireWire devices:

.. code-block::

   [2413623.229296] firewire_core 0000:09:04.0: created device fw8: GUID fa610e3f00000007, S400
   [2413623.229365] firewire_core 0000:09:04.0: created device fw11: GUID fa610e2f00000007, S400
   ...

The GUID provides the following information:

* **fa610e**\ 3f00000007: fa610e is the vendor Id, i.e. JHU LCSR
* fa610e\ **3**\ f00000007: 3 is the board Id
* fa610e3\ **f**\ 00000007: f is the FPGA board type, i.e. f for FireWire
  only, e for boards with Ethernet adapter (see controller versions)
* fa610e3f0000000\ **7**: 7 is the firmware version

.. note::

   Ubuntu 24.04 and later require sudo privileges to run ``dmesg``.
   You can change this by using ``sudo sysctl
   kernel.dmesg_restrict=0``.

``udevadm``
***********

Lastly, once the controllers are properly connected you can check all
the attributes using:

.. code-block:: bash

   udevadm info --name=/dev/fw1 --attribute-walk  | less

The output will include the info provided by ``dmesg`` and more:

.. code-block::

   looking at device '/devices/pci0000:00/0000:00:1c.4/0000:03:00.0/0000:04:00.0/fw1':
    KERNEL=="fw1"
    SUBSYSTEM=="firewire"
    DRIVER==""
    ATTR{guid}=="0xfa610e6f00000007"
    ATTR{is_local}=="0"
    ATTR{model}=="0x000001"
    ATTR{model_name}=="FPGA1/QLA"
    ATTR{units}==""
    ATTR{vendor}=="0xfa610e"
    ATTR{vendor_name}=="JHU LCSR"

The above indicates that fw1 has FPGA V1.x (no Ethernet). For FPGA
V2.x (Ethernet), the model will be 2 and the model_name will be
"FPGA2/QLA".
