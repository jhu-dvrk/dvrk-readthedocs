Mount
#####

.. note::

   If you have a full Si patient cart (i.e., SUJ with PSMs and ECM),
   you can skip this section unless you plan to remove/swap an arm.

   .. figure:: /images/Si/patient-cart-Si-JHU.jpg
      :width: 200
      :align: center

      Full Si patient cart at JHU

On a full da Vinci Si, each arm on the patient side can be easily
removed from the SUJ.  The arm is secured using 4 long bolts.

.. note::

   If the arm is folded and you can't access the bolts, you can force
   the arm to move despite the brakes.  This is not something you
   should do too often but it can help during the setup: `YouTube
   video <https://www.youtube.com/shorts/wBXQduLbHdE>`_.

Once the bolts are removed, the arm simply slides out using two
grooves on its internal walls. Please note that the passive arm has
counter weights in the patient's cart core column.  When you remove
the active arm (PSM or ECM), the corresponding SUJ passive arm will
have a tendency to move up despite the brakes.  You might want to
strap the passive arm to the bottom of the cart to prevent this.

.. figure:: /images/Si/PSM-Si-mounting-bolts.jpeg
   :width: 400
   :align: center

   Long bolts securing the PSM/ECM Si

There is a mating bracket at the end of each SUJ arm with two side
rails.  For the connections, there are two D-sub connectors (one for
motor power, one for data/LVDS) loosely mounted on the SUJ.  When the
arm is put back, the long pins on the SUJ connectors guide the arm's
D-sub connectors to make sure they properly align.

Intuitive Surgical can provide the mounting brackets so one can create
their own mount for the PSMs and ECM.  The most common brackets are
pulled from old clinical systems and are made of steel.

.. figure:: /images/Si/PSM-Si-bracket-and-80-20-plate-labeled.jpg
   :width: 400
   :align: center

   PSM/ECM Si steel mounting bracket and 80-20 plate

There are also some aluminum brackets but these should be rare.

.. figure:: /images/Si/PSM-Si-80-20-bracket-red.jpeg
   :width: 300
   :align: center

   PSM/ECM Si aluminum mounting bracket

Once you have the parts to attach the PSM/ECM to a custom frame, you
have multiple options. If you want a single PSM on a workbench or
desk, you can build a very light frame.

.. figure:: /images/Si/single-PSM-Si-with-controller.jpg
   :width: 500
   :align: center

   PSM Si on a standalone table frame

If you want to make a custom patient cart, we designed an articulated
frame for one ECM and two PSMs using 80-20 profiles.

.. figure:: /images/Si/custom-Si-patient-cart-cad.png
   :width: 400
   :align: center

   Si custom patient cart

More details can found using the following links:

* `Intuitive Research wiki
  <https://research.intusurg.com/index.php/DVRK:Topics:PSCFrameDesignSi>`_
* `CAD on onshape.com
  <https://cad.onshape.com/documents/c392f568b5f487a9c7ad416d/w/f5ef36d6be3d40eeef41b521/e/7445442fedf612c617fdd2fc>`_
  and :download:`bill of materials
  </images/Si/BOM_dVRK_Si_Short_Frame.xlsx>` (courtesy of Yilin Cai at
  Georgia Tech)
