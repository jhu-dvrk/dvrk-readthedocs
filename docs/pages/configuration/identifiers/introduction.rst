.. _identifiers:

************
Introduction
************

Each set of physical mechanical arm and controller is identified by
two numbers.

First, each arm that comes from Intuitive should be identified by its
serial number. In most cases, the serial number can be found on a
label sticked to the arm. This :ref:`serial number <serial-number>` is
used to make sure all the configuration files with arm specific
informations (e.g. potentiometer parameters) can be easily found.

Second, the FPGA (logic) boards in the dVRK controllers are identified
by their Id (i.e. board Id). This :ref:`board Id <board-id>` is used
to find a board on the communication bus used to communicate between
the PC and all the controllers (:ref:`FireWire or Ethernet
<connectivity>`). Older controllers use two board Ids, recent
controllers only use one.
