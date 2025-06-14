Calibration
###########

.. _sawrobotiocurrentcalibration:

``sawRobotIO1394CurrentCalibration``
************************************

* C+ application with a text based interface, it depends on cisst/SAW
* Compatible with **dVRK Classic controllers only** (not dVRK-Si)
* https://github.com/jhu-saw/sawRobotIO1394/tree/main/core/applications/current-calibration

This application is used to calibrate the analog-to-digital current
feedback offsets as well as the digital-to-analog offsets for the
commanded motor current. This is a required step for the initial
calibration of each set of arm and matching controller.

Usage is described in the :ref:`Classic motor current calibration
<calibration-classic-current>` section.


.. _dvrk_calibrate_potentiometers:

``dvrk_calibrate_potentiometers.py``
************************************

* ROS Python script with a text based interface, it depends on
  CRTK/dVRK Python to communicate with the dVRK system.  The dVRK
  system must be running at the same time, with the options ``-C -i
  ...``
* ROS package ``dvrk_python``
* Compatible with **dVRK Classic active arm controllers only** (not SUJ or anything dVRK-Si)
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

This application is used to calibrate the analog-to-digital conversion
from potentiometer voltages to positions in SI units. Each voltage is
converted to a position using a linear function.  This script is used
to estimate the scale and offset for the linear conversion.

Usage is described in the :ref:`Classic potentiometer calibration
<calibration-classic-pots>` section.


.. _dvrk_calibrate_potentiometer_psm:

``dvrk_calibrate_potentiometer_psm.py``
***************************************

* ROS Python script with a text based interface, it depends on
  CRTK/dVRK Python to communicate with the dVRK system.  The dVRK
  system must be running at the same time, with the options ``-C -i
  ...``
* ROS package ``dvrk_python``
* Compatible with **dVRK Classic PSM only** (not SUJ, MTM, ECM or anything dVRK-Si)
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

This application is used to calibrate the analog-to-digital conversion
from potentiometer voltages to positions in SI units. It allows the
user to calibrate the 3rd potentiometer offset and nothing else.

Usage is described in the :ref:`Classic PSM third joint potentiometer
offset <calibration-classic-pots-depth>` section.


.. _sawintuitiveresearchkitgrippercalibration:

``sawIntuitiveResearchKitGripperCalibration``
*********************************************

* C+ application with a text based interface, it depends on cisst/SAW
* Compatible with **dVRK Classic MTM only**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/gripper-calibration

This application is used to calibrate the analog-to-digital conversion
from the Hall Effect sensors used for the MTM grippers.

Usage is described in the :ref:`Classic MTM gripper calibration
<calibration-classic-gripper>` section.


.. _sawintuitiveresearchkitsipotentiometerscalibration:

``sawIntuitiveResearchKitSiPotentiometersCalibration``
******************************************************

* C+ application with a text based interface, it depends on cisst/SAW
* Compatible with **dVRK-Si PSM and ECM only**
* https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/main/core/applications/si-potentiometers-calibration

This application is used to calibrate the digital potentiometers on
the Si PSMs and ECM. It defines the home position.

Usage is described in the :ref:`Si PSMs and ECM potentiometers
calibration <calibration-si-psm-ecm-pots>` section.


.. _dvrk_calibrate_suj:

``dvrk_calibrate_suj.py``
*************************

* ROS Python script with a text based interface, it depends on
  CRTK/dVRK Python to communicate with the dVRK system.  The dVRK
  system must be running at the same time, with the options ``-s``
* ROS package ``dvrk_python``
* Compatible with **dVRK-Si SUJ only**
* https://github.com/jhu-dvrk/dvrk_python/tree/devel/scripts

This script is used to calibrate the potentiometers for the 4 SUJ arms
of the dVRK-Si.  The script relies on mechanical limits to compute the
scales and offsets used for the conversion from voltages to SI
positions.

Usage is described in the :ref:`Si SUJ potentiometers
calibration <calibration-si-suj-pots>` section.


.. _dvrk_camera_registration:

``dvrk_camera_registration.py``
*******************************

* https://github.com/jhu-dvrk/dvrk_camera_registration
