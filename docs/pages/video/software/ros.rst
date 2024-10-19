ROS
###


gscam
*****

``gscam`` is a ROS node using the ``gstreamer`` library.  The gstreamer
library supports a few frame grabbers including the Hauppage one.  The
gstreamer developement library can be installed using ``apt-get
install``.  Make sure you install gstreamer 1.0, not 0.1.

ROS Ubuntu packages vs build from source
========================================

Use ``apt install`` to install gscam on Ubuntu 18.04 with ROS 1.  The
package name should be ``ros-melodic-gscam``.  It will install all the
required dependencies for you.  ``gscam`` is also available as a
debian package for ROS2 Galatic, Humble... so use ``apt intall`` for
all ROS2 configuration.

On Ubuntu 20.04, gscam binaries for ROS 1 Noetic are not available via
``apt`` so you will need to compile it in your ROS workspace.  The
original source code is on github:
https://github.com/ros-drivers/gscam.  But you need a different
version which can be found using the pull request for Noetic Devel.
So you need to clone https://github.com/hap1961/gscam in your
``catkin_ws/src``.  Then make sure you switch to the Noetic branch:
``cd ~/catkin_ws/src/gscam; git checkout noetic-devel``.  Finally, do
``catkin build``.  This info is from June 2022, it might need to be
updated.

Using gscam
===========

To start the ``gscam`` node, we provide a couple of ROS launch scripts.
**Make sure the launch script has been updated to use a working
gstreamer pipeline** (as descrided above using ``gst-launch-1.01``).
The main difference is that your pipeline for gscam should end with
``videoconvert`` and you need to remove ``autovideosink``.

For a stereo system with the USB frame grabbers, use:

.. code-block:: bash

   sh roslaunch dvrk_robot gscam_stereo.launch rig_name:=jhu_daVinci

Where ``jhu_daVinci`` is a name you want to give to your camera rig.  This
name will be used to define the ROS namespace for all the data
published.  It is also used to define a directory to save the results
of your camera calibration or load said camera calibration
(i.e. ``dvrk_robot/data/<rig_name>``).  If you don't have a calibration
for your rig, you can still render both video channels using the ROS
topics:

* ``/jhu_daVinci/left/image_raw``
* ``/jhu_daVinci/right/image_raw``

For a system with a Decklink Duo, the `gscam_config` in a launch script would look like:

.. code-block:: xml

   <param name="gscam_config" value="decklinkvideosrc connection=sdi device-number=0 ! videoconvert"/>

(rqt\_)image_view
*****************

One can use the ``image_view`` node to visualize a single image:

.. code-block:: bash

   rosrun image_view image_view image:=/jhu_daVinci/right/image_raw


If you prefer GUI, you can use ``rqt_image_view``, a simple program to
view the different camera topics.  Pick the image to display using the
drop-down menu on the top left corner.

dVRK launch files
*****************

At that point, we assume most groups are using SDI frame grabbers and
the stereo display is using two images.  The following launch files
will start the frame grabbers using ``gscam`` so one can collect the
images over ROS and create 2 windows (left and right views) that can
be used in your stereo display (drag and drop).  The launch files are
in ``dvrk_video`` and are provided for both ROS1 (xml) and ROS2
(Python).

* ``decklink_stereo_1280x1024.launch``: launches 2 ``gscam`` nodes to grab
  the video and de-interlace it as well as two ROS image viewers that
  can be dragged on the HRSV
* ``decklink_stereo_goovis.launch``: launches 2 ``gscam`` nodes to grab
  the video, de-interlace and start 2 OpenGL based windows that can be
  dragged on the stereo display.  The main advantage of the OpenGL
  views are:

  * Low latency since there is no ROS message passing involved
  * Ability to change aspect ratio (e.g. with :ref:`Goovis HMD
    <goovis>`)
