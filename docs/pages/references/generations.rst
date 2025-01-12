.. _davinci-generations:

*******************************
Generations of da Vinci systems
*******************************

Models
######

.. csv-table:: da Vinci Generations
   :name: da-vinci-generations
   :header: "Model", "Year", "Surgeon's console", "PSM/ECM/USM", "Setup Joints", "Endoscope"
   :align: center

   "Classic [1]_ ", "2000", "ver 1 (640x480)", "ver 1 (PSM/ECM)", "ver 1", "ver 1 with SD"
   "S       ", "2006", "ver 1 (640x480 or 1024x768)", "ver 2 (PSM/ECM)", "ver 2", "ver 1 with SD or HD"
   "Si      ", "2009", "ver 2 HD", "ver 3 (PSM/ECM)", "ver 2", "ver 2 HD"
   "X [2]_  ", "2017", "ver 2 HD", "ver 4 (USM)    ", "ver 2", "ver 3 HD"
   "Xi [2]_ ", "2014", "ver 2 HD", "ver 4 (USM)    ", "ver 3", "ver 3 HD"
   "5 [2]_  ", "2024", "ver 3 ", "ver 4 (USM)    ", "ver 3", "?"

.. [1] Initially known as IS1200 and later on Standard

.. [2] Versions of components for the X, Xi and 5 are based on
       physical appearance. It is quite possible the internals are
       radically different from one version to another.

dVRK support
############

Supported:

* Classic and S MTMs (ver1) with QLA based arm controllers
* Classic PSMs and ECMs (ver1) with QLA based arm controllers
* Classic SUJ (ver1) with QLA based SUJ controller
* Si PSMs (ver2) and ECMs (ver3) with dRAC based arm controllers
* S and Si SUJ (ver2) with dESSJ and dRAC based arm controllers

Not supported:

* S PSMs (ver1) and ECMs (ver2). Some S system came with more recent
  PSMs or ECMs which are supported. The version is identified by the
  number following **ASSY,PSM<ver>** or **ASSY,ECM<ver>** on the arm's
  label.

  .. figure:: /images/Si/ECM-Si-serial-number-label.jpeg
     :align: center
     :width: 400

     dVRK supported S/Si ECM, **ASSY,ECM3**

  .. figure:: /images/Si/PSM-Si-serial-number-label.jpeg
     :align: center
     :width: 400

     dVRK supported S/Si PSM, **ASSY,PSM3**

* Si patient cart motorized base
* Si MTMs
* Anything X, Xi or 5
