********
Ethernet
********

Important Notes
###############

* The Ethernet adapter on the computer must be configured for "Link
  Local" on Linux (not static IP nor DHCP)
* The network cable goes directly from the computer to the "bridge"
  controller (no hub nor switch)
* The software then communicates using UDP, the "bridge" controller is
  a UDP server and the computer is a UDP client

Setup
#####

Ethernet adapter
****************

You will need a network adapter dedicated to the communication with
the dVRK controllers (e.g., a PCIe Ethernet adapter).  The dVRK
network port on the computer can't be plugged in to a router or hub
and used to access the Internet.  Therefore, we recommend to install 2
network adapters on your computer, one for the LAN/WAN and one for the
dVRK.  The dVRK dedicated network adapter will be directly connected
to one of the dVRK controllers on the FireWire chain.  Please avoid
having dVRK controllers connected to 2 different computers through
Ethernet.

We recommend a built-in network adapter (e.g., a PCIe Ethernet
adapter).  We don't have any specific recommendation for the chipset,
just make sure it is supported by Linux.  USB3/USB-C network adapters
might work too but we don't have extensive experience with these.

Configuration
*************

You will need to configure the dVRK dedicated network adapter to use
"Link-Local Only"

Ubuntu
======

Start the application ``sudo nm-connection-editor`` (this should work on Ubuntu from 18.04 to 24.04).  Select the Ethernet adapter you want to configure for the dVRK:

* In tab "Ethernet", change MTU to 3000.  The default is 1500 and is not enough if you have a full da Vinci (2 MTMS, 3 PSMs, ECM and SUJ).
* In tab "IPv4 Settings", change "method" to "Link-Local Only"

MacOS
=====

Running the dVRK on MacOS is experimental and not that useful.  This
being said, there is no network configuration required on MacOS.
Somehow the OS figures out that the adapter should be configured for
Link-Local by itself.

Windows
=======

Running the dVRK on Windows is experimental and currently not very stable. There is no network configuration required because it defaults to "Link-Local" if it cannot be configured using DHCP.

Virtual machines
================

We recommend a native OS (as opposed to a virtual machine guest OS) as
we're not totally sure how the VM network adapter would impact
performances.  If you need to use a VM, the following has been used
for a Linux guest with a Windows host using VMWare:

* Select "Bridged" in virtual machine network settings and check the box "connected directly to the physical network"
* Follow instructions from https://kb.vmware.com/s/article/1020359

Testing connectivity
####################

First you should make sure your Ethernet port is properly configured.  On Linux and MacOS you can use ``ifconfig``.  The output for the dVRK dedicated adapter should look like:

.. code-block::
   
   eno1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 3000
        inet 169.254.62.229  netmask 255.255.0.0  broadcast 169.254.255.255
        inet6 fe80::902e:58f9:b2c9:dec3  prefixlen 64  scopeid 0x20<link>

The IP address (``inet``) should start with ``169.254`` for "Link Local".  The netmask should be ``255.255.0.0``.

``qladisp``
***********

The dVRK utility ``qladisp`` can communicate with the controllers over UDP using the command line option ``-pudp``.  For example ``qladisp -pudp`` should return something like:

.. code-block::
   
   Trying to detect boards on port:
   Server IP: 169.254.0.100, Port: 1394
   Broadcast IP: 169.254.255.255, Port: 1394
   Using interface eno1 (2), MTU: 3000
   Firewire bus reset, FPGA = 25, PC = 0
   InitNodes: Firewire bus generation = 25
   InitNodes: found hub board: 7
   BasePort::ScanNodes: building node map for 16 nodes:
     Node 0, BoardId = 6, Firmware Version = 7
     Node 1, BoardId = 7, Firmware Version = 7
     Node 2, BoardId = 0, Firmware Version = 7
     Node 3, BoardId = 1, Firmware Version = 7
   BasePort::ScanNodes: node 4 is not a QLA board (data = 0)
   BasePort::ScanNodes: found 4 boards

``qladisp`` provides some feedback specific to the UDP port (as
opposed to FireWire):

* Name of interface used: ``eno1``
* MTU: ``3000`` (or whatever you've set)
* UDP server IP: ``169.254.0.100`` (default in software, but can be
  changed, for example by specifying ``-pudp:169.254.0.50``)
* UDP server port: ``1394`` (hard coded in firmware, picked for no
  other reason than it's the other name for FireWire)
* Message ``node 4 is not a QLA board`` indicates that another
  FireWire device is connected to the FireWire chain (i.e. not a dVRK
  controller).  In this case, a PC is still connected.  This can lead
  to issues so it is recommended to unplug the computer from the
  FireWire chain.
 
``ping``
********

The dVRK controllers (with firmware 7+) also support ICMP requests.
To test the communication, you can ``ping`` the controllers using
``ping 169.254.0.100`` (or whatever IP address has been configured).
The output should look like:

.. code-block::
   
   PING 169.254.0.100 (169.254.0.100) 56(84) bytes of data.
   64 bytes from 169.254.0.100: icmp_seq=1 ttl=64 time=0.247 ms
   64 bytes from 169.254.0.100: icmp_seq=2 ttl=64 time=0.308 ms
   ...

This allows to check that you can reach the controllers over Ethernet
and that the loop time is reasonable (~0.3 ms).
