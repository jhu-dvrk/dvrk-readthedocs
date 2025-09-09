.. _user-computers:

Computers
#########

.. warning::

   You will need at least one PC running Linux, **but** it is strongly
   recommended to use two or more if you also need to set up the video
   pipeline or if you plan to run some heavy computations along the
   dVRK code.

   Since we use ROS extensively, it is easy to spread the computing
   load across computers (see
   http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

.. warning::

   If you use ROS bags for your data collection, this process should
   also be running on a different PC (not the dVRK PC).

Hardware
********

PC
==

For the PC connected to the dVRK controller itself, we recommend:

* 8 cores minimum, ideally hyper-threaded (Intel i7, i9 or equivalent Xeon)
* 16 GB RAM minimum, 32 recommended
* SSD drive for the OS and home directories

dVRK connections
================

* FireWire adapter (required)

  * A dedicated FireWire controller for each chain of controllers -
    you can hook up to 8 Classic controllers (16 FPGA/QLA boards) in a
    single chain so 1 FireWire controller is fine for most users. A
    full Classic system with MTML, MTMR, PSM1, PSM2, PSM3, ECM and SUJ
    requires 13 boards and can run on a single FireWire port!

  * If possible, choose a card with a native PCIe FireWire
    chipset. Avoid adapters with a PCI based FireWire chipset and a
    PCI to PCIe bridge.

    * Good FireWire adapters:

      * `SYBA Low Profile PCI-Express FireWire
        <https://www.amazon.com/gp/product/B002S53IG8/>`_. This card
        comes with a regular and a low profile plate, so it can also be
        used in low profile and full size desktop computers. It has
        been used extensively at Johns Hopkins
      * Early dVRK users have tested different cards and the SIIG
        FireWire adapter NN-E20012-S2 works well (uses a TI chipset)

    * Bad FireWire adapters:

      * `StartTech pex1394a2v2
        <https://www.startech.com/en-us/cards-adapters/pex1394a2v2>`_

  * You can also check the following document, but it's a bit old and
    outdated:
    http://support.presonus.com/hc/en-us/article_attachments/203654243/Compatible_Hardware_List_7-12.pdf. Any
    card from the compatible list should work.

* Ethernet adapter (optional)

  * A second Ethernet adapter is recommended if you plan to connect
    the dVRK controllers to the PC using Ethernet instead of FireWire
    (see :ref:`connectivity <connectivity>`).

  * PCIe adapters are recommended over USB network adapters.  If
    you're using a USB adapter, make sure it's at least USB3.

  * PCIe cards with multiple ports should be PCIe X2 lanes.  1X cards
    might not have enough bandwidth for the multiple Ethernet ports.

Visualization
=============

These might be installed on the second PC.

* Graphic card

  * Remember, if possible, use a different computer for the video
    pipeline (i.e. endoscopic images)
  * If you plan to send images to the stereo display, you will need
    two extra VGA outputs for the standard CRTs (or HDMI/DisplayPort
    with VGA converters) or two DVI outputs for the flat panels (see
    `ISI private Wiki
    <http://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Topics:StereoViewerLCD>`_)
  * In general Nvidia cards work fine on Linux but...

    .. warning::

       Make sure the drivers from NVIDIA are properly installed, "nouveau" drivers tend to disrupt the FireWire communication.  This happens fairly often due to EUFI/Secure Boot option.  To make sure your drivers are installed correctly, try ``nvidia-smi``.

* Frame grabbers

  * If you've received a stereo camera with an endoscope from
    Intuitive Surgical, you might want to buy a dual SDI frame
    grabber.  The `Blackmagic DeckLink
    Duo2 <https://www.blackmagicdesign.com/products/decklink/models>`_
    have been used extensively by the dVRK community.  Remember to
    also buy some good SDI cables.
  
OS
**

Software and OS requirements:

* dVRK 2.4 and main

  * ROS 2 (**recommended**):

    * Ubuntu 24.04 (**recommended**), 22.04 or 20.04
    * ROS Jazzy on Ubuntu 24.04 (**recommended**), Galactic on Ubuntu 20.04, and Humble on Ubuntu 22.04

  * ROS 1:

    * Ubuntu 20.04
    * ROS Noetic on Ubuntu 20.04

  
* dVRK up to 2.3.1:

  * ROS 1:

    * Ubuntu 18.04 or 20.04
    * ROS Melodic on Ubuntu 18.04 and Noetic on Ubuntu 20.04

  * ROS 2:

    * Ubuntu 20.04 or 22.04
    * ROS Galactic on Ubuntu 20.04 and Humble or Iron on Ubuntu 22.04

* dVRK 1.7 (older systems, only if you need to run older firmware)

  * Ubuntu 16.04 or 18.04 (64 bits of course)
  * ROS 1: Kinetic on Ubuntu 16.04 on Melodic on Ubuntu 18.04
