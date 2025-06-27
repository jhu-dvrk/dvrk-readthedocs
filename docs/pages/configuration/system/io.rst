.. _config-io:

.. include:: /includes/logic-view-console.rst

IOs
***

**Schema based documentation**

.. raw:: html

    <iframe style="width: 100%; height: 70vh" frameBorder="1" src="../../../schemas/dvrk-IO.html"></iframe>


**Additional info**

* For the ``port``, see also :ref:`connection between the PC and the
  dVRK controllers <connectivity>`.

* If you want to use the ``port`` ``udp``, all your controllers need
  to have a `FPGAs V3 <fpga>` (any controller released after 2024) and
  you need to daisy chain your controllers with Ethernet cables.  You
  don't need any FireWire cable nor adapter on the PC.

* Regarding the possible conflicts between two MTMRs or MTMLs, see
  :ref:`board Id <board-id>`.

* For the IO period, see :ref:`software architecture <threads>` since
  the IO and PID often share the same thread (and periodicity).

* For the extra ``configuration_files``, see available :ref:`dMIB IOs
  <dmib-io>` on the dVRK Classic controllers.
