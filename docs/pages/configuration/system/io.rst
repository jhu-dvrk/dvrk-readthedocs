.. _config-io:

.. include:: /includes/logic-view-console.rst

IOs
***

Schema based documentation
==========================

.. raw:: html

    <iframe style="width: 100%; height: 70vh" frameBorder="1" src="../../../schemas/dvrk-IO.html"></iframe>


Additional info
===============

* For the ``port``, see also :ref:`connection between the PC and the
  dVRK controllers <connectivity>`.

* If you want to use the ``port`` ``udp``, check you need `FPGAs V3
  <fpga>`.  cables and use ``udp``.

* Regarding the possible conflicts between two MTMRs or MTMLs, see
  :ref:`board Id <board-id>`.

* For the IO period, see :ref:`software architecture <threads>` since
  the IO and PID often share the same thread (and periodicity).

* For the extra ``configuration_files``, see available :ref:`dMIB IOs
  <dmib-io>` on the dVRK Classic controllers.
