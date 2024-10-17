Sterile adapter
***************

The PSMs have the ability to detect when a sterile adapter or tool is
installed using digital inputs.  For this to work, you will need to
short a couple of pins on your sterile adapters (left two pins on the
photo).

Make sure you place the wire to short the two pins as deep as possible
and keep the extremities of the pins as clean as possible.  If the tip
of the pins are covered with soldering, you might have issues with the
electric contact when inserting the adapter.  This also affects the
ability to read the instrument's Dallas chip (see :ref:`instrument
detection <dallas>`).

The dVRK controller should be able to reliably detect the sterile
adapter.  To check if the sterile adapter is properly detected,
monitor the *Buttons* widget in the *IO* tab of the dVRK console GUI
(see :ref:`widget example <io>`).


.. figure:: /images/Classic/PSM/classic-PSM-sterile-adapter-back.jpeg
   :width: 600
   :align: center

   Modified Classic sterile adapter (clean soldering)

.. figure:: /images/Classic/PSM/modified-sterile-adapter.jpg
   :width: 400
   :align: center

   Modified Classic sterile adapter (a bit messy)
