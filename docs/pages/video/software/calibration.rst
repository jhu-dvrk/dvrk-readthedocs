Calibration
###########

Create a catkin package
***********************

With ROS1, the simplest way to save and retrieve the results of a
camera calibration is to create a dummy ROS package.  The package name
will allow other ROS nodes to find the results of the camera
calibration later on.  As far as we know, there is no equivalent in
ROS2.

Stereo
======

You will need to create a new package to store the calibration results.  At that point, choose a name for your camera (aka stereo rig).  For this example, we will use "jhu_daVinci":

.. code-block:: bash

   cd ~/catkin_ws/src
   catkin_create_pkg jhu_daVinci
   catkin build
   source ~/catkin_ws/devel/setup.bash # you have a new package so you need to source again


Mono
====

There is no difference but for sake of demonstration, we will create the package under a different name, say "depstech", a familiar brand of cheap USB cameras.

.. code-block:: bash

   cd ~/catkin_ws/src
   catkin_create_pkg depstech
   catkin build
   source ~/catkin_ws/devel/setup.bash # you have a new package so you need to source again

Start the video
***************

Stereo
======

The following is based on dVRK provided launch files to capture the
stereo video using Decklink SDI frame grabbers.  If your frame
grabbers are different you will have to create your own launch files.
Note that you have to provide the name of your stereo rig:

.. code-block:: bash

   roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=jhu_daVinci

Mono
====

The following example is based on a video4linux compatible source:

.. code-block:: bash

   roslaunch dvrk_video gscam_v4l.launch camera_name:=depstech

Calibrate the camera
********************

Stereo
======

The calibration is performed using the ROS provided application,
please refer to their documentation for the parameters
(http://wiki.ros.org/camera_calibration).  You need to make sure the
video stream is started and you are using the correct rig name.

.. code-block:: bash

   rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 12x10 --square 0.0045 right:=/jhu_daVinci/right/image_raw left:=/jhu_daVinci/left/image_raw left_camera:=/jhu_daVinci/left right_camera:=/jhu_daVinci/right

The command line above assumes you're using a 12x10 calibration grid
and the size of each square is 4.5mm.  Once the calibration is
performed, don't forget to save and commit using the GUI.  You should
now have two new files in the catkin package you created for the
stereo rig under ``calibrations``: ``left.yaml`` and ``right.yaml``.

You can find some calibration checkerboards ready to print in the
``assets`` directory.  Make sure the scale is correct after printing.
The simplest solution is to measure 10 consecutive squares and verify
that it's 10 times the claimed square size in mm.  Also to note, the
checkerboard size (e.g. 12x10) is based on the number of corners
between the squares.  For example, the 12x10 actually has 13x11
squares.

Mono
====

The only difference is the parameters:

.. code-block:: bash

   rosrun camera_calibration cameracalibrator.py --size 12x10 --square 0.0045 image:=/depstech/image_raw camera:=/depstech


Restart the video using the calibration
***************************************

At that point, you need to stop the launch file used for the video
acquisition and restart it with the ``stereo_proc`` parameter set to
``True``.  This will add a ROS node to compute the rectified images
and publish the camera parameters needed for the camera registration.

Stereo
======

.. code-block:: bash

   roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=jhu_daVinci stereo_proc:=True


Mono
====

.. code-block:: bash

   roslaunch dvrk_video gscam_v4l.launch camera_name:=depstech mono_proc:=True

RViz
****

Use RViz to display both channels at the same time.  Add image, select
topic and then drop image to separate screen/eye on the HRSV display.
You can save your settings so everytime you start RViz you will have
both images.

References:

* http://wiki.ros.org/camera_calibration
* http://wiki.ros.org/camera_calibration_parsers
