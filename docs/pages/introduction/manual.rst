***************
Getting started
***************

Accounts
========

For all users who have access to a dVRK, you will need to create two
accounts to access some privates resources:

* Intuitive Surgical hardware wiki http://research.intusurg.com/dvrk:
  please contact Intuitive to get an account created for each active
  user.  This wiki contains some important documentation regarding the
  hardware, including unboxing instructions and wiring fixes.
* Google group for dVRK users
  https://groups.google.com/d/forum/research-kit-for-davinci and
  research-kit-for-davinci@googlegroups.com We strongly encourage
  users to send questions using this google group.

  How to add new user:

  1. Go to
     https://groups.google.com/forum/#!forum/research-kit-for-davinci
  2. Use the *Apply for membership* link to request membership.
     **Don't forget to mention your group/university so the group
     admin can identify you**.

Other readings
==============

Make sure you read all the other documentation, before you get
started.

* ISI Research wiki (account required): https://research.intusurg.com/dvrkwiki/index.php?title=DVRK:Docs:Main

  * dVRK User Manual
  * dVRK Unpacking Guide
  * S Console Disassembly and Vision Testing for the dVRK

* da Vinci User Manuals.  These are extremely useful to understand how
  the real da Vinci should be setup and used:

  * :download:`da Vinci Standard <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-classic-user-manual.pdf>`
  * :download:`da Vinci Si <https://dvrk.lcsr.jhu.edu/downloads/manuals/davinci-si-user-manual.pdf>`


Overview
========

The usual steps are:

* **Compile the software with ROS** - This has to be performed once
  per user, each user should maintain their own version of the
  software in their home directory.  The core software for the dVRK
  can be built without ROS but we strongly recommend you use ROS and
  the catkin build tools.

  * :ref:`ROS 1 <ros1>`
  * :ref:`ROS 2 <ros2>`

* **Controller connectivity** - This has to be performed once per
  computer.  The goal is to make sure you have the proper hardware and
  OS configuration to communicate with the controllers over
  :ref:`FireWire <firewire>` or :ref:`Ethernet <ethernet>`.

* **Configuration** - :ref:`The configuration step <configuration>`
  has to be performed once per robotic arm.  Once the configuration
  has been generated, we strongly recommend you save all your
  configurations files to share between the users in your group.  We
  maintain a GitHub organization (https://github.com/dvrk-config) to
  host the configuration files of different groups, let us know if you
  want a repository for your files! (contact Anton Deguet @ JHU)

* **Hardware setup** - :ref:`The hardware setup section <setup>`
  describes the few hardware modifications required and physical
  connections between the controllers and the arms as well as between
  the controllers and the PC.

* **Calibration** - Once the controllers are physically connected, the
  software has been compiled and you have the base configuration
  files, you can proceed to the :ref:`calibration steps <calibration>`
  to identify some values specific to each arm and controller pair.
  The process depends on the generation of arms:

  * :ref:`Classic/Standard arms <calibration_classic>`
  * :ref:`Si arms <calibration_si>`

  This should be a one time step.  Don't forget to save the results of
  the calibration under git.

* **Applications** - :ref:`The applications section <applications>`
  shows how to run the different examples provided with the dVRK.
  Once your system is set up and calibrated, you will mostly use the
  ROS ``dvrk_robot dvrk_console_json`` node.  We also provide a few
  debugging and calibration utilities.

* **Usage** - todo

* **Development** - :ref:`The development section <devel>` shows how
  to write your own applications on top of the dVRK software stack.
