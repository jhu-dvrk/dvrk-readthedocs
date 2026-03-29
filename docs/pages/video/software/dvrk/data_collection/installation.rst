Installation & Prerequisites
============================

Before starting your data collection pipeline, assume your environment has been successfully built according to the :ref:`ROS 2 Compilation <ros2>` guide.

The primary dependencies should already be covered natively:

- **GStreamer (gi)**: Core video extraction and formatting logic.
- **opencv-python**: Utilities consumed structurally by the extraction arrays.
- **rclcpp / rosbag2_cpp**: The backend logging frameworks powering the ROS 2 decoupled architecture.

NVIDIA Hardware Acceleration
----------------------------

To use NVIDIA GPUs for hardware-accelerated video encoding (NVENC), you need to have the NVIDIA drivers and the GStreamer ``nvcodec`` plugin installed. This is often provided by the NVIDIA DeepStream SDK.

.. code-block:: bash

    sudo apt update
    sudo apt install libnvidia-encode-*

To verify availability, run:

.. code-block:: bash

    gst-inspect-1.0 nvh264enc

If you just installed DeepStream or new GStreamer plugins and they are not appearing in ``gst-inspect-1.0``, you may need to clear the GStreamer registry cache:

.. code-block:: bash

    rm -rf ~/.cache/gstreamer-1.0

Installation & Environment
--------------------------

Before running the application, ensure your ROS2 environment is sourced:

.. code-block:: bash

    source /opt/ros/YOUR_DISTRO/setup.bash
    # or if using a workspace
    source install/setup.bash

To build the workspace:

.. code-block:: bash

    colcon build --symlink-install
