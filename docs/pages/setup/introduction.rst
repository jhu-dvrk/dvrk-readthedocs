.. _setup-intro:

************
Introduction
************

For the original dVRK kit, Intuitive Surgical provided two PSMs, two
MTMs, a stereo display, a foot pedal tray and sometimes a stereo
endoscope.  Users were expected to find a way to mount these are on
some kind of frame, usually relying on 80/20 aluminum profiles (or
equivalent).

Later on, some groups acquired full decommissioned da Vinci Classic
(donation, eBay...), including the surgeon's console and the patient's
cart with its setup joints (SUJ, passive arms supporting the PSMs and
ECM). These groups didn't need to build any frame to support the
active arms and needed to interface some existing components found on
a full system: :ref:`head sensor <head-original>`, SUJ, :ref:`focus
controller <focus-original>`...

Lately, Intuitive Surgical has been sending gutted Classic and S
surgeon's console with two MTMs, the foot pedal tray and SVGA stereo
displays. If this is the case, there is no need to build a frame and
one might want to take advantage of the head sensor, focus controller,
:ref:`console height adjustment <height>`...

On the patient's side, the SUJs, PSMs and ECMs provided by Intuitive
now come from decommissioned da Vinci Si systems.  Some group will
receive a full patient cart (with SUJ) while others will receive only
the active arms and will therefore need to figure out how to mount
them.

.. note::

   As of 2024, there is no dVRK support for the Si/Xi surgeon's
   console.  So what is commonly referred to as a "dVRK Si" is usually
   built around the da Vinci Si patient's side (SUJ, PSMs, ECM) and a
   da Vinci S surgeon's console.

To add to the complexity, Intuitive has over the time donated roughly
3 types of :ref:`endoscopic cameras <video>` and these can be mounted
on any generation of ECM.

As such, the steps required to set up a dVRK totally depends on which
:ref:`hardware is available on each site <dvrk-examples>`.  The only
exception is the safety chain (E-Stop) for the dVRK controllers.

The following sections describe how to connect the :ref:`E-Stop chain
<estop>` and modifications required to set up components from the da
Vinci :ref:`Classic <setup-classic>` and :ref:`Si <setup-si>`.
