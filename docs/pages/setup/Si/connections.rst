.. _setup-si-connections:

Connections
###########

For a system without SUJ, we provide two custom cables with D-sub
connectors at both ends.  At one end, they plug in the arm itself (see
paragraph below) and the other end is connected to the :ref:`dVRK-Si
controllers <controller-si-exterior>`.

For users with an original patient cart (SUJ), the connectors for the
active arms are not accessible.  Therefore, we use the internal wiring
in the SUJ arm to bring all the signals to the base of the cart.  An
additional card (dSIB) is used to interface between the internal
connectors at the base of the cart and the dVRK controllers (see
:ref:`Si SUJ <setup-si-suj>`).

.. figure:: /images/Si/Si-connections-overview.png
   :width: 600
   :align: center

   Overview of wiring between the active arms and the controllers in 3
   scenarios: original, dVRK with and without SUJ

For all custom frames, the mounting brackets have an open space at the
bottom for the D-sub connectors.  The connectors can hold without
screws, but you might want to add some kind of strain relief since the
cables are pretty heavy.

.. figure:: /images/Si/PSM-Si-back-connectors.jpeg
   :width: 400
   :align: center

   Standard D-sub connectors on a PSM Si (same as ECM Si)

.. figure:: /images/Si/PSM-Si-SUJ-connectors.jpeg
   :width: 400
   :align: center

   Standard D-sub connectors at the end of the SUJ passive arm

.. figure:: /images/Si/PSM-Si-80-20-connectors.jpeg
   :width: 300
   :align: center

   Custom dVRK cables connected under a PSM Si
