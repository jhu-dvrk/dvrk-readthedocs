.. _usage:

#####
Usage
#####

Choose a session by the patient-side hardware and the console you will use.
The simulation examples include physical operator devices: real MTMs with
Newton, and an OpenXR headset and controllers with PyBullet.

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Session
     - Operator input
     - Patient side
     - Video
   * - :ref:`Real robot <usage-real>`
     - MTMs, pedals, head sensor
     - Physical PSMs and ECM
     - Endoscope to HRSV or another stereo display
   * - :ref:`MTMs with Newton <usage-mtms-newton>`
     - Physical MTMs, pedals, head sensor
     - Simulated PSMs and ECM
     - Newton camera to HRSV
   * - :ref:`OpenXR with PyBullet <usage-openxr-pybullet>`
     - Headset and tracked controllers
     - Simulated PSMs and ECM
     - PyBullet camera to the headset
   * - :ref:`OpenXR with a real patient cart <usage-openxr-real>`
     - Headset and tracked controllers
     - Physical PSMs and ECM
     - Endoscope to the headset
   * - :ref:`Force Dimension with Isaac Sim <usage-force-dimension-isaac>`
     - Haptic masters and joystick pedals
     - Simulated PSMs and ECM
     - Isaac Sim stereo video
   * - :ref:`Haply with a real patient cart <usage-haply-real>`
     - Two Haply devices, original pedals, Goovis head sensor
     - Physical Si PSMs and ECM
     - Endoscope to the Goovis HMD

The hardware examples use JHU configuration files with specific serial numbers,
calibrations, IO ports, and video devices. Use the configuration matching your
own hardware as described under :ref:`Real robot <usage-real>`.

.. toctree::

   usage/real
   usage/simulation
   usage/other
   usage/tips
