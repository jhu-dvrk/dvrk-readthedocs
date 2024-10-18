.. _config-focus:

Focus controller
****************

See also :ref:`dMIB IOs <dmib-io>`.

Once you've build :ref:`your custom cable <focus-original>`, you can
modify your console JSON configuration file and add:

.. code-block:: JSON
		
    "endoscope-focus": {
        // replace the MTML part in the filename by
        // whatever controller you're connecting the cable to
        "io": "io/sawRobotIO1394-MTML-dv-endoscope-focus.xml"

The example above assumes that:

* You're using the dVRK software **rev 1.6** or above
* You have connected to the camera focus cable to the MTML controller.
  If you're connecting the endoscope focus unit to another controller
  and you can't find the corresponding configuration file in
  ``sawIntuitiveResearchKit/share/io``, feel free to create one and
  contribute it back to the community
* You have the da Vinci foot pedal connected with the Camera +/-
  toggle pedal properly working.  You can check in the Qt graphical
  user interface, under the IO/Buttons tab.

At that point, you should be able to control the camera focus using
the foot pedals.  When pressing the +/- pedal you should:

* See the focus change
* See the "Focus In"/"Focus Out" LED turn on/off on the vision cart
* Hear the motor on the camera head

The da Vinci Focus controller uses joint limit switches to stop the
motion when the focus mechanism reaches a limit.  When this happens,
the motor will stop and the "Focus In" or "Focus Out" LED will start
blinking.

See also :ref:`dMIB IOs <dmib-io>`.
