Configurator Tool
=================

Tuning the stereoscopic pipeline is executed interactively via the bundled Python-based ``configurator`` script within the `dvrk_display` architecture.

Launching the Configurator
--------------------------

To execute a fresh mapping, supply an existing (or entirely new) JSON file directly via the runtime arguments:

.. code-block:: bash

   ros2 run dvrk_display configurator -c my_settings.json

Alternatively, display left and right eyes directly to entirely decoupled window panes using the ``-s`` or ``--split`` argument (highly advantageous on modern HDR tiled displays):

.. code-block:: bash

   ros2 run dvrk_display configurator -c my_settings.json -s

Interactive Controls
--------------------

When mounted, the system listens for physical keystrokes overriding the composition in real-time. Text overlays reflect the updated states dynamically across either integrated or `--split` views:

- **Arrow Keys**: Shift the internal crop horizontally and vertically across the stereo projection geometry (``horizontal_shift_px``/``vertical_shift_px``).
- **``+ / -``**: Dynamically inflate or constrict the resolution crop dimensions (1% step modifiers).
- **``r``**: Immediately lock and revert shift and crop parameters to their factory defaults.
- **``f``**: Fullscreen hardware override.
- **``q`` (or ``ESC``)**: Safely exit from the viewer layout and instantly serialize all configuration offsets securely back to the active JSON context target.

.. note::
   Proper geometric alignment (shift and crop) requires the calibration jig provided by Intuitive Surgical. The jig presents a known pattern through the endoscope optics, allowing the operator to precisely align the left and right eye images for correct stereoscopic viewing.

Color Calibration Tuning
------------------------

To counteract asymmetrical lens fading or disparate hardware exposures, pressing the ``c`` keystroke immediately iterates the rendering arrays through sophisticated visual color-matching profiles:

1. **None**: Neutral pass-through (Default).
2. **Left Ref**: Instructs the target configurator engine to statistically map and balance the Right eye's hue, saturation, and exposure explicitly against the Left eye distributions.
3. **Right Ref**: Vice versa.
4. **Average**: Dynamically interpolates an exact mathematical average of both sources and applies correction to left/right simultaneously.

Because calculations happen on-the-fly iteratively across sub-frames, you immediately see the updated rendering. The final HSV states automatically flush their bounds down into JSON (`left_color` / `right_color`) upon exit.
