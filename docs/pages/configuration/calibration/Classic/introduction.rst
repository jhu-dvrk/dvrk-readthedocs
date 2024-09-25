Introduction
************

.. warning::

   This section is for the Classic arms only!

Before you get started, make make sure you've setup the dVRK arms first:

* :ref:`Hardware setup <setup-classic>`
* :ref:`Connectivity <connectivity>`
* :ref:`Configuration files <configuration-overview>`

The dVRK Classic arms use a fair amount of analog signals that need to
be calibrated for better accuracy.  The initial values in the XML IO
configuration file are generated from the ``.cal`` file provided by
ISI.  Unfortunately, these values don't take into account the
differences between the original ISI controllers and the dVRK
controllers so it is required to calibrate each set of arm/controller.

There are a number of different calibration steps to perform,
different for each type of arm. The relevant calibrations for each arm
are listed below.  The "single step" links point to different
procedures, make sure you follow all the steps for each arm.

.. csv-table:: Calibration steps for da Vinci Classic
   :name: default-board-ids
   :header: "", "PSM{1,2,3}", "MTM{L,R}", "ECM", "SUJ"
   :align: center

   "Motor current offsets",  ":ref:`single step<calibration-classic-current>`", ":ref:`single step<calibration-classic-current>`", ":ref:`single step<calibration-classic-current>`", ":ref:`single step<calibration-classic-current>`"
   "Brake current offsets",  "", "", ":ref:`single step<calibration-classic-current-brakes>`", ""
   "Potentiometer scales", ":ref:`single step<calibration-classic-pots-scale>`", ":ref:`single step<calibration-classic-pots-scale>`", ":ref:`single step<calibration-classic-pots-scale>`", ":ref:`single step<calibration-classic-suj-pots>`"
   "Potentiometer offsets", ":ref:`step 1<calibration-classic-pots-offset>` and :ref:`step 2<calibration-classic-pots-depth>`", ":ref:`single step<calibration-classic-pots-offset>`", ":ref:`single step<calibration-classic-pots-offset>`", ":ref:`single step<calibration-classic-suj-pots>`"
   "MTM gripper", "", ":ref:`single step<calibration-classic-gripper>`", "", ""
   "Brake release current", "", "", ":ref:`single step<calibration-classic-ecm>`", ""
   "Gravity compensation", "", ":ref:`single step<calibration-classic-mtm-gc>`", "", ""
