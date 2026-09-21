.. include:: /includes/logic-view-soft-bridge.rst

.. _devel-bridges:

.. _devel-middleware:

*******************
Middleware bridges
*******************

The dVRK system communicates with external applications through middleware
bridges. All bridges are not specific to the dVRK: they can be applied to any
`cisst/SAW component
<https://cisst.readthedocs.io/en/main/libraries/overview.html>`_.

For a conceptual overview of how bridges fit in the software architecture see
:ref:`Bridges <middleware>`.

.. toctree::

   bridges/ros
   bridges/socket-streamer
   bridges/igtlink
