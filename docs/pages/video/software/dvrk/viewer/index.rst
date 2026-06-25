.. _stereo-viewer:

Stereo Viewer
=============

.. warning::
   The dVRK stereo viewer and associated `calibration` tools are exclusively supported on ROS 2. ROS 1 is **not** supported.

The ``dvrk_display`` package provides a lightweight, pure C++ application designed to intercept, compose, and display stereo video feeds using native GStreamer elements. You can launch the viewer using the ``stereo`` (or ``mono``) executable, for example: ``ros2 run dvrk_display stereo``.

See a video demonstration of the stereo viewer and its overlay features on the `@dvrk-robot <https://www.youtube.com/@dvrk-robot>`_ YouTube channel: `dVRK Stereo Display Example <https://youtu.be/NyHr2-rO-T0?si=KeEKETHXyHFg2tmx>`_.

GStreamer Native Rendering & Low Latency
----------------------------------------

Using GStreamer directly provides dramatically lower latency compared to routing massive, high-definition stereo streams entirely through ROS architectures. At its core, the viewer constructs direct OpenGL hardware displays to drive the master console natively.

The architecture still provides immense modular flexibility by isolating separate viewer stages into distinct executables via ``unixfd`` inter-process video routing. Because ``unixfd`` strictly passes local descriptors natively, all video rendering and frame grabbing **must** be executed on a single, unified computer. 

In addition to rendering the video feed natively, the viewer displays discrete icons and textual overlays to cleanly visualize the operational state of the overarching :ref:`dVRK System Node <system>` within the master console. However, because the layout overlays depend strictly on lightweight data strings, the master dVRK system node can safely exist on an entirely separate remote computer—the viewer will naturally extract the console's state seamlessly utilizing its integrated ROS 2 topic listener.

Running the Viewer
------------------

The package provides two executables: ``stereo`` for 3D displays (such as the surgeon's console) and ``mono`` for standard flat panels. They can be launched via ROS 2:

.. code-block:: bash

   ros2 run dvrk_display stereo -c <config.json> [--grid] [-g <0|1|2|3>]
   ros2 run dvrk_display mono -c <config.json> [-g <0|1|2|3>]

Command-Line Options
~~~~~~~~~~~~~~~~~~~~

* **``-c <config.json>``**: (Required) Path to the JSON configuration file.
* **``--grid``**: (Optional, ``stereo`` only) Displays a calibration grid and white circle overlay to help align/fuse the display horizontally.
* **``-g <level>`` / ``--dot <level>``**: (Optional) Dumps the constructed GStreamer pipeline graph as a Graphviz ``.dot`` file. To use this feature, the ``GST_DEBUG_DUMP_DOT_DIR`` environment variable must be set (e.g. ``export GST_DEBUG_DUMP_DOT_DIR=/tmp``).
  
  Available detail levels:
  
  * ``0``: Minimal (elements only)
  * ``1``: Light (elements and states, recommended)
  * ``2``: Medium (adds media types)
  * ``3``: Full (all parameters)

Environment Variables
~~~~~~~~~~~~~~~~~~~~~

* **``GDK_BACKEND``**: The viewer relies on ``gtkglsink`` for GTK-embedded OpenGL hardware rendering, which requires OpenGL via GLX. Under Wayland sessions, the viewer automatically forces ``GDK_BACKEND=x11`` to run via XWayland unless it is already set in the environment. You can manually override this if needed.

.. toctree::

   configuration
   calibration_tool
   pipeline

