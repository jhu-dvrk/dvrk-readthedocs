Connections
***********

The only cable to connect for the Classic MTMs, PSMs and ECM is the
large cable with an ITT Cannon 156 connector that comes from the arm.

When connecting to the dVRK controller, make sure you don't force the
connector in, the connector should insert without any forces (ZIF -
Zero Insertion Force). The direction varies between arms, cable coming
out either on the left or right of the dVRK controller.

Once the connector is inserted, you need to use a large flat head
screw driver to secure the it in place.  The screw on the back shell
is connected to a camshaft to lock the connector to the controller.
You need to turn by 90 degrees at most (a quarter turn), clockwise to
lock and counterclockwise to unlock.  If you need to remove the
connector, don't forget to unscrew it first.

.. figure:: /images/controllers/Classic-ITT-Cannon-screw.jpeg
   :width: 400
   :align: center

   ITT Cannon back shell screw

.. warning::

   Failure to lock/screw the arm connector will lead to bad contacts
   and will prevent the arm from working correctly.


Arm specific notes:

* The MTM cables are fairly short so you will need to place the
  :ref:`dVRK Classic controllers <controllers-classic>` either on the
  surgeon's console or on its side. If you place the controllers next to
  the console, it is recommended to place them on the left side so the
  ITT Cannon cables are not bent too tight.
* The cables coming from the PSMs and ECM are reasonably long so you
  can place your :ref:`dVRK Classic controllers <controllers-classic>`
  in a rack close to the robotic arms.  See :ref:`examples at Johns
  Hopkins <dvrk-examples>`.

  .. figure:: /images/Classic/SUJ/SUJ-Classic-connections.jpeg
     :width: 600
     :align: center

     Classic SUJ connections, SUJs connected to the single SUJ
     controller (black box) and active arms connected to the 4 dVRK
     Classic controllers in rack.

* For the SUJ, don't use the very long cables that were originally
  between the patient's cart and the surgeon's console. Connect the
  SUJs directly to the dVRK SUJ controller!  When everything is
  connected, all the connectors at the base of the patient cart should
  be empty.

  .. figure:: /images/Classic/patient-cart-arm-suj-plugs.jpg
     :width: 400
     :align: center

     Empty connectors on Classic patient cart

  The cables from the SUJs are pretty short but they should reach the
  back of dVRK SUJ controller when it is mounted on the patient cart's
  column.
