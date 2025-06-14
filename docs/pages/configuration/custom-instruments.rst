.. _config-custom-instruments:

Custom instruments
##################

Introduction
************

There are two main reasons why someone would want to define their own
instrument parameters:

* New mechanical design
* Using an existing instrument but change it's behavior (e.g. change
  control point, depth limit)

.. caution::

   Make sure you know what you're doing when modifying instrument
   definition files.  This can lead to unexpected motions on the
   manipulators and physical damage.

.. caution::

   Starting with the dVRK 2.0 and firmware 6, it is possible to
   :ref:`automatically detect the instrument <config-dallas>` using
   the Dallas chip.  If the instrument you're using is not a standard
   instrument (either mechanical or parameter changes), you won't be
   able to use the automatic instrument detection and you will have to
   revert to either fixed or manual instrument specification.

Main files
**********

Some examples are provided under
``dvrk_config_jhu/jhu-dVRK/custom-tool``.  You should open
https://github.com/dvrk-config/dvrk_config_jhu/tree/main/jhu-dVRK/custom-tool
in a separate tab to browse through the example files.

The main files are:

* **System file:** See comments in
  ``system-PSM1-Custom-Dummy.json``.  By default the system
  application will infer the arm configuration file name from the
  ``name`` and ``serial`` fields by concatenating them
  (``<name>-<serial>.json``).  In the example for a custom instrument,
  we added two fields.

  #. ``system`` is used to help locate configuration files for the
     arm, namely the ``sawRobotIO1394-<name>-<serial>.xml`` by adding
     ``share/<system>`` to the search path.
  #. ``arm`` to overwrite the default rule to locate the arm
     configuration file.  We recommend to follow the convention
     ``<name>-<serial>-<instrument-name>.json`` to name your new arm
     file (e.g. ``PSM1-28007-Custom-Dummy.json``).

* **Arm file:** See comments in ``PSM1-28007-Custom-Dummy.json``.
  There are three fields used for custom instruments:

  #. ``custom-tool-index`` is used to point to your list of custom
     instruments.  This list will be added to the default list of
     instruments loaded by the system
     (i.e. ``share/tool/index.json``).
  #. ``kinematic`` can be used to overwrite the default behavior of
     the PSM itself (first 3 joints).  This can be used to change the
     joint limits on the first 3 joints based on the design of your
     custom instrument.  This is a very specific case and we expect
     most users should use the default, i.e. ``kinematic/psm.json``.
  #. ``tool-detection`` should be set to either ``MANUAL`` or
     `FIXED``.  If set to ``MANUAL``, you will need to provide the
     tool name in the configuration file using the convention
     ``<name>:<model>``.  The custom ``name`` and ``model`` are
     defined in the instrument index file (see below).

* **Instruments index file:** See comments in
  ``custom-tool-index.json``.  This file contains a list of custom
  ``instruments`` with:

  #. ``model``: 6 digits number, usually start with ``40`` for Classic
     instruments and ``42`` for S instruments.  We recommend to
     preserve this naming convention for your custom instruments and
     use the last 4 digits to identify your instrument.  See also
     ``generation`` field below.
  #. ``names``: All caps names with underscores.  These names are used
     by the dVRK software/firmware to identify the instruments with a
     somewhat human readable string.  The software allows multiple
     names (aliases) to support different spellings from the Dallas
     chip.  Since automatic detection is not compatible with custom
     instruments, you should only provide one name.
  #. ``description``: Name that will appear on the Qt GUI and in logs.
  #. ``generation``: Generation can be either ``Classic`` or ``S``.
     This is used to determine at which depth the instrument should be
     engaged or not (estimating if the instrument is inside the
     cannula or not) and how close to the RCM point the instrument can
     go in cartesian mode.  If you are designing your own instrument
     based on a Classic (or S) instrument and the shaft's length is
     unchanged, use the ``generation`` from the donor instrument.
  #. ``file``: File with path relative to the ``share`` directory
     containing the actual instrument's definition.

* **Instrument definition file:** See comments in
  ``CUSTOM_DUMMY_400900.json``.  File containing the DH parameters for
  the instrument, actuator to joint coupling matrix, position and
  torque joint limits...  See examples in ``share/tool``.

* **Manipulator definition file:** See comments in
  ``psm-Custom-Dummy.json``.  File containing the DH parameters for
  the PSM itself, i.e. first 3 joints.  This file should be based on
  the default PSM kinematics found in ``share/kinematic/psm.json``.
  This might be useful if you need to change the joint limits to avoid
  collisions with your custom instruments.  Most users probably won't
  need to create their own version of this file.

Examples
********

Change the control point
========================

In your custom instrument definition file, change the last column of the ``tooltip-offset``:

.. code-block:: JSON

   "tooltip-offset" : [[ 0.0, -1.0,  0.0,  0.0 ],
                       [ 0.0,  0.0,  1.0,  0.01],  // yaw to tip in meters, about 10 mm
                       [-1.0,  0.0,  0.0,  0.0 ],
                       [ 0.0,  0.0,  0.0,  1.0 ]]

Change jaw's angles (grasping torque)
=====================================

When control in position, the jaws maximum grasping torque is based on
the PID output.  To increase the PID output without changing the PID
gains and get a stronger grasp, the trick is to decrease the minimum
joint limit.  This way, when the jaws stop moving because they're in
contact with the object to grasp, the tracking error will increase and
so will the grasping force.  The jaws' joint limit (``qmin``) is
defined in the instrument definition file:

.. code-block:: JSON

   "jaw" : {
        // for last joint, manual says [0, 30] but we need -40 to allow stronger torque, 80 to open wide
        "qmin": -0.698132, // -40 degrees - overriding ISI values
        "qmax":  1.39626,  //  80 degrees - overriding ISI values
        "ftmax": 0.16
    }

We suggest lowering the limit by small increments until you reach the
grasping force you need.  This will work up to a point since the
torque itself is capped using the ``ftmax`` value defined for the
``jaw`` in the instrument definition file.  The following is not
recommended but you can also increase the value for ``ftmax``.  Keep
in mind than the IO level will then apply another cap defined in the
``sawRobotIO1394-PSMx-xxxxx.xml`` file (defined in amps).

Disable the engage procedure
============================

If you've designed a very delicate instrument or added delicate
apparatus on your instrument, you might want to make sure the
instrument doesn't get engaged when inserted (sequence of motion
applied to the last 4 actuators to mate the sterile adapter disks with
the instrument).  To do so, you can modify the
``tool-engage-position`` in the instrument definition file:

.. code-block:: JSON

   "tool-engage-position" : {
        "lower" : [-0.0, -0.0, -0.0, 0.0],
        "upper" : [ 0.0,  0.0,  0.0, 0.0]
   }

In a normal sequence of events, the user should add the sterile
adapter first.  At that point the software will rotate the last 4
actuators back and forth to engage the adapter's disks.  At the end of
that sequence, all 4 actuators will move to 0.  Then the user will add
the instrument.  Since the actuators are all at 0 and the
``tool-engage-position`` defines a range from 0 to 0, the last 4
actuators will not move.

New mechanical design
=====================

When you design your new instrument, there might be a few different issues to keep in mind.

* Can the kinematic be described using DH parameters for a serial manipulator?

  * If so, the built-in numerical solver might be usable and so will
    the cartesian control modes.  This will allow you to use the
    built-in tele-operation component.
  * If the kinematic is very specific (e.g. snake like, parallel
    platform...), integration will be trickier.  To get started, we
    suggest you ignore the cartesian API and only use the joint API.
    You can then implement your own forward/inverse kinematic in a
    separate ROS node (C++/Python/Matlab) and send/receive joint
    states to/from the dVRK system.  If you need a tighter level of
    integration, you will need to derive the
    ``mtsIntuitiveResearchKitPSM`` class.  Reach out to the dVRK
    developers if you get to that point.

* Is your instrument a full 6 dof instrument?

  * If it is, just make sure the ``DH`` parameters match your instrument.
  * If your instrument is not a full 6 dof instrument, not all
    cartesian position will be reachable (not event taking into
    account joint limits).  Since the internal numerical solver is not
    able to handle this case, we suggest creating some virtual links
    at the end of your kinematic chain.

    * Define virtual links at end of kinematic chain with position
      offsets set to 0.
    * Coupling matrix must be invertible so keep some ones on the
      diagonal for the virtual joints.

* Is there a jaw on your instrument?

  * If there is, just make sure the ``jaw`` parameters match your
    instrument.
  * If there isn't, we're going to pretend there is one:

    * Set all the ``jaw`` parameters to 0.
    * Make sure the last element on the diagonal of the actuator to
      joint coupling is set to 1.
