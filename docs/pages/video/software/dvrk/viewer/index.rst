.. _stereo-viewer:

Stereo Viewer
=============

.. warning::
   The dVRK stereo viewer and associated `configurator` tools are exclusively supported on ROS 2. ROS 1 is **not** supported.

The ``dvrk_stereo_viewer`` package provides a lightweight, pure C++ application designed to intercept, compose, and display stereo video feeds using native GStreamer elements. 

GStreamer Native Rendering & Low Latency
----------------------------------------

Using GStreamer directly provides dramatically lower latency compared to routing massive, high-definition stereo streams entirely through ROS architectures. At its core, the viewer constructs direct OpenGL hardware displays to drive the master console natively.

The architecture still provides immense modular flexibility by isolating separate viewer stages into distinct executables via ``unixfd`` inter-process video routing. Because ``unixfd`` strictly passes local descriptors natively, all video rendering and frame grabbing **must** be executed on a single, unified computer. 

In addition to rendering the video feed natively, the viewer displays discrete icons and textual overlays to cleanly visualize the operational state of the overarching :ref:`dVRK System Node <system>` within the master console. However, because the layout overlays depend strictly on lightweight data strings, the master dVRK system node can safely exist on an entirely separate remote computer—the viewer will naturally extract the console's state seamlessly utilizing its integrated ROS 2 topic listener.

.. toctree::

   configuration
   configurator_tool
   pipeline
