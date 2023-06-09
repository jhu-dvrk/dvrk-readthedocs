<!--ts-->
   * [Introduction](#introduction)
   * [Credit / Citation](#credit--citation)
   * [Links](#links)
      * [Community](#community)
      * [Core Software](#core-software)
      * [Software Ecosystem](#software-ecosystem)
   * [Updates](#updates)
      * [Firmware](#firmware)
      * [Software](#software)
   * [Acknowledgments](#acknowledgments)

<!-- Added by: anton, at: 2021-02-25T10:06-05:00 -->

<!--te-->

# Introduction

The da Vinci Research Kit (**dVRK**) is an “open-source mechatronics” system, consisting of electronics, firmware, and software that is being used to control research systems based on the now retired first-generation da Vinci system from [Intuitive Surgical Inc](https://www.intuitive.com/).  You can find a more detailed description of the dVRK on the [Intuitive Foundation's site](https://www.intuitive-foundation.org/dvrk/).  The dVRK is now deployed in close to [40 differents institutions worldwide](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline).

![Full da Vinci system with dVRK controllers at the Johns Hopkins University (LCSR)](/jhu-dvrk/sawIntuitiveResearchKit/wiki/assets/general/full-daVinci-B08-03-2021.png)

The sawIntuitiveResearchKit folder provides several example applications for controlling the Research Kit for the da Vinci System using the [IEEE-1394 (FireWire) controller](http://jhu-cisst.github.io/mechatronics/). The picture above shows a full da Vinci system at JHU that uses the dVRK controllers.

# Credit / Citation

If you use the dVRK in your research, please cite the following paper:

  P. Kazanzides, Z. Chen, A. Deguet, G. S. Fischer, R. H. Taylor, and S. P. DiMaio, “[An open-source research kit for the da Vinci(R) surgical system](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014.pdf),” in IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014, pp. 6434–6439. [BibTeX](/jhu-dvrk/sawIntuitiveResearchKit/wiki/kazanzides-chen-etal-icra-2014)

For posters and videos, please include the [dVRK logo](https://github.com/jhu-dvrk/dvrk-logo) if possible.

# Links

## Community

* [News](/jhu-dvrk/sawIntuitiveResearchKit/wiki/News) from the dVRK community
* [Groups](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Timeline) and deployment timeline
* [Videos](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Videos) of the dVRK in action
* [Publications](/jhu-dvrk/sawIntuitiveResearchKit/wiki/Publications)
* Resources:
  * Intuitive Surgical hardware wiki: http://research.intusurg.com/dvrk
  * Intuitive Foundation: http://www.intuitive-foundation.org/dvrk/ (includes online applications to get retired da Vinci components from Intuitive Surgical)
  * Private Slack channel JHU dVRK: https://jhudvrk.slack.com/
  * Private Google group: https://groups.google.com/d/forum/research-kit-for-davinci and research-kit-for-davinci@googlegroups.com (**for dVRK users only**; use the Google group web page to request membership, don't forget to mention your group/university so the group admin can identify you)
  * *cisst* libraries: http://github.com/jhu-cisst/cisst/wiki
  * Johns Hopkins University Mechatronics: http://jhu-cisst.github.io/mechatronics
  * List of all JHU LCSR Software: http://jhu-lcsr.github.io/software/

## Core Software

The software applications use some or all of the following [cisst](https://github.com/jhu-cisst/cisst/wiki)/SAW components (and Qt widgets):
* [mtsRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394) - interface to IEEE-1394 (FireWire) controller boards
* [mtsPID](https://github.com/jhu-saw/sawControllers) - PID controller used for MTM and PSM robots
* [mtsTeleoperation](https://github.com/jhu-saw/sawIntuitiveResearchKit) - Teleoperation components
* [mtsTextToSpeech](https://github.com/jhu-saw/sawTextToSpeech) - Text to speech component (for warning and error messages)

Github build status:
* ROS 1: ![Ubuntu 20.04 ROS Noetic](https://github.com/jhu-dvrk/dvrk-github-workflow/workflows/Ubuntu%2020.04%20ROS%20Noetic/badge.svg) ![macOS 10.15](https://github.com/jhu-dvrk/dvrk-github-workflow/workflows/macOS%2010.15/badge.svg)
* ROS 2: ![Ubuntu 20.04 ROS Galactiic](https://github.com/jhu-dvrk/dvrk-github-workflow/workflows/Ubuntu%2020.04%20ROS%20Galactic/badge.svg) [![Ubuntu 22.04 ROS Humble](https://github.com/jhu-dvrk/dvrk-github-workflow/actions/workflows/ubuntu-22-04-ros-humble.yaml/badge.svg)](https://github.com/jhu-dvrk/dvrk-github-workflow/actions/workflows/ubuntu-22-04-ros-humble.yaml)


The core components are written in C++ and can be compiled on most OSs (Linux, Windows, MacOS).  Before the release of the dVRK software 2.0 and firmware 7, FireWire (more specifically `libraw1394`) was required to communicate with the dVRK controllers so it made little or no sense to compile on dVRK on anything but Linux.  We now support UDP over Ethernet Link Local so it is now possible to control the dVRK using Windows or MacOS. This being said, we rely heavily on ROS for most applications so it makes a lot more sense to keep using Linux and more specifically Ubuntu.

A [ROS 1](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild) interface is available via [mtsROSBridge](https://github.com/jhu-cisst/cisst-ros) base class and [dVRK ROS programs and files](https://github.com/jhu-dvrk/dvrk-ros).  [ROS 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2) is partially supported but not extensively tested.

## Software Ecosystem

Several groups have developed software modules that integrate with the dVRK and may be useful to others in the community. Many of these software modules use ROS to interface to the dVRK.

### Simulators
* [Asynchronous Multi-Body Framework (AMBF)](https://github.com/WPI-AIM/ambf) - dynamic simulator developed at Worcester Polytechnic Institute (WPI); includes models of the dVRK manipulators and interfaces to the dVRK hardware (e.g., to use MTMs as input devices).
* [V-Rep Simulator for the dVRK](https://github.com/unina-icaros/dvrk-vrep) - V-Rep simulator developed at University of Naples.
* [ATAR](https://github.com/neemoh/ATAR) - Bullet based dynamic simulator developed at Politecnico di Milano.

### Machine Learning
* [dVRL](https://github.com/ucsdarclab/dVRL) - reinforcement learning environment, based on V-Rep, developed at University of California, San Diego.
* [AMBF-RL](https://github.com/WPI-AIM/ambf_rl) - reinforcement learning environment, based on AMBF, developed at Worcester Polytechnic Institute (WPI).
* [UnityFlexML](https://gitlab.com/altairLab/unityflexml) - machine learning environment, based on Unity 3D, developed at University of Verona.
* [SurRoL](https://github.com/med-air/SurRoL) - reinforcement learning environment, based on PyBullet, developed at The Chinese University of Hong Kong.

### Mixed Reality
* [dVRK-XR](https://github.com/jhu-dvrk/dvrk-xr) - mixed reality visualization, based on Unity 3D, developed at Johns Hopkins University; interfaces to dVRK via UDP or ROS.

### High Level Control
* [Mesh based virtual fixtures](https://github.com/mli0603/PolygonMeshVirtualFixture) - haptic feedback with Slicer visualization, via [OpenIGTLink](https://github.com/jhu-saw/sawOpenIGTLink), developed at Johns Hopkins University.

### Data Recording/Playback
* [DVRK_RecordAndPlayback](https://github.com/careslab/DVRK_RecordAndPlayback) - data recording and playback developed at Wayne State University.

### Autonomous Camera Control
* [autocamera](https://github.com/careslab/autocamera) - autonomous camera control developed at Wayne State University.

### Computer Vision - TBD
* Instrument tracking
* Stereo reconstruction

# Updates

## Firmware

Firmware version 6 or 7 is now required (dVRK 2.0+), please upgrade your firmware to 7 unless you want to use the software versions 1.7.x and 2.x simultaneously.  Version 7 adds support for Ethernet/UDP for [FPGA 2+](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/Board-Versions) and many [other changes](https://github.com/jhu-cisst/mechatronics-firmware/compare/Rev6...Rev7). See
https://github.com/jhu-cisst/mechatronics-firmware/wiki/FPGA-Program for step-by-step instructions to upgrade your firmware.

## Software

* *August 2021:* Version 2.1.0 released:
  * [sawIntuitiveResearchKit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/CHANGELOG.md) (main dVRK code)
  * [dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros/blob/master/CHANGELOG.md) (ROS specific dVRK code)
  * [sawRobotIO1394](https://github.com/jhu-saw/sawRobotIO1394/blob/master/CHANGELOG.md) (FireWire/Ethernet IO component)
* *April 2021:* Version 2.0.0 released
* *July 2019*: Version 1.7.1 released
* *April 2019*: Version 1.7.0 released
* *May 2018*: Version 1.6.0 released
* *November 2017*: Version 1.5.0 released
* *August 2016*: Version 1.4.0 released
* *January 2016*: Version 1.3.0 released
* *October 2015*: Version 1.2.0 released
* *April 2015*: Version 1.1.0 released
* *April 2014*: Moved to GitHub
* *May 2013*: Initial Public Release

# Acknowledgments

The cisst software has been developed with the support of the National Science Foundation, EEC 9731748, EEC 0646678, and MRI 0722943.

The da Vinci Research Kit was supported by the National Science Foundation, via the National Robotics Initiative (NRI), as part of the collaborative research project "Software Framework for Research in Semi-Autonomous Teleoperation" between The Johns Hopkins University (IIS 1637789, 2016-2020), Worcester Polytechnic Institute (IIS 1637759, 2016-2021), and the University of Washington (IIS 1637444, 2016-2020).

The *dVRK Consortium*, based at JHU, has been organized by members of the dVRK community to provide technical support.

The National Science Foundation is supporting a network of networks, including the dVRK network, via the AccelNet program, as part of the collaborative project "International Collaboration to Accelerate Research in Robotic Surgery" led by The Johns Hopkins University (OISE 1927354, 2019-2024) and Worcester Polytechnic Institute (OISE 1927275, 2019-2024).

Starting with Version 2.0.0, the dVRK software has also been supported by the Multi-Scale Medical Robotics Center (MRC), InnoHK, Hong Kong, China.

