.. _config-teleops:

.. include:: /includes/logic-view-soft-teleop.rst

Teleoperation
*************

.. warning::

   Make sure you can detect the :ref:`operator's presence
   <config-head>`.

You can define multiple tele-operation components for different pairs
of MTM/PSMs. You can use the same PSM or MTM in different
teleoperation pairs (see console :ref:`component
<console-component-teleops>` and :ref:`API<api-console>`).

.. code-block:: JSON

   "psm-teleops":
   [
     {
       "mtm": "MTMR",
       "psm": "PSM1"
     }
   ]
   ,
   "ecm-teleop":
   {
     "mtml-left": "MTML",
     "mtmr-right": "MTMR",
     "ecm": "ECM"
   }

Note that "ecm-teleop" requires an ECM.  This feature is
available in version 1.6.0 and above.

For release **1.6** and above, there is a new scope for tele-operation
components called "configure-parameter". In this scope, one can define
the teleoperation scale, misc. thresholds and other configuration
parameters. For example, if one wants to disable all jaw motions:

.. code-block:: JSON

   "psm-teleops":
   [
     {
       "mtm": "MTMR",
       "psm": "PSM1",
       "configure-parameter": {
         "ignore-jaw": true
       }
     }
   ]
