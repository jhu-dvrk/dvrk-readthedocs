Frame grabbers
##############

Frame grabbers and ROS
**********************

This describes a fairly low cost setup that can be used with the dVRK
HRSV display (High Resolution Stereo Video).  We use a couple of cheap
USB frame grabbers (Hauppage Live 2) for the analog videos from SD
cameras (640x480).  For HD systems (720p and 1080i), we tested a
BlackMagic DeckLink Duo frame grabber with dual SDI inputs.  For
displaying the video back, we just use a graphic card with two spare
video outputs.  The software relies heavily on ROS tools to grab and
display the stereo video.  Some lag is to be expected.

The general steps are:

* Make sure the frame grabber works (e.g. using tvtime or vendor
  application)
* Figure out the gstreamer pipeline and test using ``gst-launch-1.0``
* Create a launch file for gscam with the gstreamer pipeline you just
  tested

.. note::

   This page is a collection of notes that might be helpful for the
   dVRK community, but it is in no way exhaustive.  If you need some
   help re. gstreamer and gscam, you should probably start searching
   online and/or reach out to the gstreamer and gscam developers.

PCIe Blackmagic DeckLink Duo SDI frame grabber
**********************************************

We recommend these frame grabbers if your endoscopic camera has SDI
outputs (see :ref:`S HD <camera-s-hd>` and :ref:`Si <camera-si>`
endoscopes).  A single card can be used to capture both channels of
the stereo stream.

You first need to install the drivers from BlackMagic, see
https://www.blackmagicdesign.com/support/family/capture-and-playback
The drivers are included in the package "Desktop Video".  Once you've
downloaded the binaries and extracted the files from BlackMagic,
follow the instructions on their ReadMe.txt.  For 64 bits Ubuntu
system, install the ``.deb`` files in subfolder ``deb/x86_64`` using
``sudo dpkg -i \*.deb``.

If your card is old, the DeckLink install might ask to run the
BlackMagic firmware updater, i.e. something like
``BlackmagicFirmwareUpdater update 0``.  After you reboot, check with
``dmesg | grep -i black`` to see if the card is recognized.  If the
driver is working properly, the devices will show up under
``/dev/blackmagic``.

You can quickly test the frame grabber using ``MediaExpress`` which
should be installed along the drivers.  You can also select the video
input using ``BlackmagicDesktopVideoSetup`` (also installed along
drivers).

If you need to remove all the BlackMagic packages to test a different
version, use ``sudo apt remove desktopvideo* mediaexpress*``.

To test if the drivers are working and the cards are working, use
gstreamer 1.0 or greater.  You will also need the proper gstreamer
plugins installed:

.. code-block:: bash

   sudo apt install gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad


Once gstreamer is installed, you can use a few command lines to test
the drivers:

* ``gst-inspect-1.0 decklinkvideosrc`` will show you the different
  parameters for the DeckLink gstreamer plugin
* ``gst-launch-1.0`` can be used to launch the streamer and pipe it to
  see the video live on the computer.  For example, we used
  ``gst-launch-1.0 -v decklinkvideosrc mode=0 connection=sdi
  device-number=0 ! videoconvert ! autovideosink``.

  * ``mode=0`` is for auto-detection and is optional
  * ``connection=sdi`` is to force to use an SDI input if your card
    has different types of inputs.  This is optional.
  * ``device-number=0`` is to select which input to use if you have
    multiple inputs

* On a DeckLink Duo, we found that one can see the stereo video using two text terminals:

    * ``gst-launch-1.0 decklinkvideosrc device-number=0 ! videoconvert ! autovideosink``
    * ``gst-launch-1.0 decklinkvideosrc device-number=1 ! videoconvert ! autovideosink``



USB SD S-Video frame grabbers
*****************************

.. warning::

   These frame grabbers use an analog video from the early
   century (~2000) and the quality is not great.  If you can, try to
   upgrade to HD CCUs and a SDI frame grabber (see above).

The frame grabbers we used most often for SD endoscopes are Hauppage USB Live 2:

 * Manufacturer:
   http://www.hauppauge.com/site/products/data_usblive2.html
 * Amazon, about $45:
   http://www.amazon.com/Hauppauge-610-USB-Live-Digitizer-Capture/dp/B0036VO2BI

When you plug these in your PC, make sure both frame grabbers are on
different USB channels otherwise you won't have enough bandwidth to
capture both left and right videos.  To check, use ``lsusb``.  The
output should look like:

.. code-block:: bash
		
   $> lsusb
   Bus 004 Device 006: ID 0461:4e22 Primax Electronics, Ltd
   Bus 004 Device 005: ID 413c:2107 Dell Computer Corp.
   Bus 004 Device 004: ID 0424:2514 Standard Microsystems Corp. USB 2.0 Hub
   Bus 004 Device 003: ID 2040:c200 Hauppauge
   Bus 004 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
   Bus 004 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
   Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
   Bus 001 Device 002: ID 2040:c200 Hauppauge
   Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
   Bus 003 Device 002: ID 8087:0024 Intel Corp. Integrated Rate Matching Hub
   Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub

In this example, the Hauppage frame grabbers are on bus ``004`` and
``001``.

To check if the frame grabbers are working, one can use ``tvtime``
(available on most Linux distributions).  The two frame grabbers
should appear in ``/dev`` with the prefix ``video``.  For example:

.. code-block:: bash
		
   $> ls /dev/video*
   /dev/video0  /dev/video1


The numbering (i.e. which frame grabber is ``/dev/video0`` and which
one is ``/dev/video1``) depends on the order the grabbers are plugged
in.  To have a consistent ordering, always plug the frame grabbers in
the same order, e.g. first the left channel and then the right
channel.  Alternatively, you can set up ``udev`` rules to automatically
assign a device name for a specific frame grabber identified by serial
number (see below).

Some Linux distributions might restrict access to the video devices
using the ``video`` group.  To check, do:

.. code-block:: bash
		
   ls -l /dev/video*

If the result shows something like:

.. code-block::
   
   crw-rw----+ 1 root video 81, 0 Nov 14 11:47 /dev/video0

you will need to add your user id to the ``video`` group.  Do not use
``sudo tvtime``, ``sudo`` might work for ``tvtime``, but it's not going to work
with ``gscam``.  You should fix the Unix file permissions first and make
sure you can access the video without ``sudo``.

To test each channel one after another:

.. code-block:: bash
		
   tvtime -Ld /dev/video0

Then:

.. code-block:: bash
		
   tvtime -Ld /dev/video1

Once in ``tvtime``, change the input to S-Video by pressing ``i`` key.
If you see a black image, it's possible that you don't have enough
light in front of your camera or endoscope.  If you happen to use a
real da Vinci endoscope and CCUs (Camera Control Units), you can use
the toggle switch ``CAM/BAR`` to use the video test pattern
(https://en.wikipedia.org/wiki/SMPTE_color_bars).

Using the color bar is also useful to test your video connections,
i.e. if your video is noisy or not visible, put the CCUs in bar mode.
If the video is still not working, the problem likely comes from your
S-video cables.  If the color bars show correctly, the problem comes
from the cables to the endoscope or the endoscope itself.

Once you have the video showing in tvtime, you need to figure out the
gstreamer options.  There is some information online, and you can use
``gst-inspect-1.0`` (see more details in DeckLink Duo section above).
You can also use the command line tool ``v4l2-ctl`` to figure out the
output format of your frame grabber.  The option ``-d0`` is to specify
``/dev/video0``, if you're using a different device, make sure the
digit matches.  Example of commands:

.. code-block:: bash
		
   v4l2-ctl -d0 --get-fmt-video
   v4l2-ctl -d0 --list-formats-ext

On Ubuntu 18.04, we found the following syntax seems to work with the
USB Hauppage Live2:

.. code-block:: bash
		
   gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,interlace-mode=interleaved ! autovideosink

To setup a ``udev`` rule, you first need to find a way to uniquely
identify each frame grabber.  To start, plug just one frame grabber
then do ``ls /dev/video*``.  Use the full path to identify each frame
grabber (e.g. ``/dev/video0``, ``/dev/video1``...):

.. code-block:: bash
		
   udevadm info --attribute-walk /dev/video0

Scroll through the output to find the serial number:

.. code-block:: bash

   ATTR{manufacturer}=="Hauppauge"
   ...
   ATTR{serial}=="0011485772"

Note that this info should correspond to the messages in ``dmesg -w``
when you plugged your frame grabber.  Now we can create a ``udev`` rule
to automatically assign the frame grabber to a specific ``/dev/video``
"device".  You can write the rules in
``/etc/udev/rules.d/90-hauppauge.rules`` using sudo privileges, replace
the serial numbers with yours, the following example is for a stereo
system:

::
   
   SUBSYSTEM=="video4linux", ATTRS{manufacturer}=="Hauppauge", ATTRS{serial}=="0011367747", SYMLINK+="video-left"
   SUBSYSTEM=="video4linux", ATTRS{manufacturer}=="Hauppauge", ATTRS{serial}=="0011485772", SYMLINK+="video-right"

Save the file and then do ``sudo udevadm control --reload-rules`` to
apply the rules.  No need to reboot the computer, just unplug your
frame grabber, wait a few seconds, re-plug it and then do ``ls -l
/dev/video*`` to confirm that the rule worked.  If this didn't work,
these pages have some useful info for debugging ``udev`` and
``video4linux`` rules:

* https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux
* https://unix.stackexchange.com/questions/424887/udev-rule-to-discern-2-identical-webcams-on-linux
