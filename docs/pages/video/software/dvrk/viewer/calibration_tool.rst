Calibration Tool
================

The ``calibration`` script is an interactive Python tool for tuning the
``dvrk_display`` stereo pipeline.  It previews the live stereo feed in an
OpenCV window and writes all calibrated values back into the JSON configuration
file on exit.

Launching the Calibration Tool
------------------------------

Supply an existing (or new) configuration file with the ``-c`` flag.  The left
and right camera streams are read from the config; you can override them
temporarily with ``-l`` and ``-r``:

.. code-block:: bash

   ros2 run dvrk_display calibration -c my_settings.json

Display each eye in a separate window with ``-s`` / ``--split`` (recommended for
Gooviz or other side-by-side displays):

.. code-block:: bash

   ros2 run dvrk_display calibration -c my_settings.json -s

To oscillate a PSM arm while adjusting the display offset (useful for depth
verification), add ``-p <PSM name>``:

.. code-block:: bash

   ros2 run dvrk_display calibration -c my_settings.json -p PSM1

Interactive Controls
--------------------

All keys operate in real time.  The on-screen status line shows current values.
On exit (``q``), the tool prompts whether to save the updated configuration.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Key
     - Action
   * - **← / →** (arrow keys)
     - Shift the camera alignment horizontally (``camera.alignment.horizontal_shift_px``
       ±2 px per press).  Moves the left and right crop windows in opposite directions.
   * - **↑ / ↓** (arrow keys)
     - Shift the camera alignment vertically (``camera.alignment.vertical_shift_px``
       ±2 px per press).
   * - **``+`` / ``-``**
     - Increase / decrease the crop size by 1 % of the original frame (adjusts
       ``camera.crop``).
   * - **``o`` / ``p``**
     - Stretch the crop width out / in by 2 px to correct non-square pixel aspect
       ratios independently of the percentage zoom.
   * - **``g``**
     - Toggle the calibration grid and circle overlay used for display-depth alignment.
   * - **``[`` / ``]``**
     - Decrease / increase ``display_horizontal_offset_px`` by 2 px per press.
       Use with the grid enabled to find the offset that fuses the grid at the
       surgeon's working depth.
   * - **``r``**
     - Reset camera alignment shifts (``horizontal_shift_px`` and
       ``vertical_shift_px``) to zero.
   * - **``c``**
     - Cycle through color-calibration modes (see `Color Calibration Tuning`_ below).
   * - **``v``**
     - Toggle PSM arm oscillation (requires ``-p`` / ``--psm`` on launch).
   * - **``f``**
     - Toggle fullscreen for the preview window.
   * - **``q``**
     - Quit and prompt to save the configuration.

Step 1 — Camera Alignment
--------------------------

**Goal:** make a physical calibration cross (placed at a known depth in front of
the endoscope) appear at zero disparity — i.e., the left and right images of the
cross fuse perfectly in the overlay window.

**How:**

1. Point the endoscope at the calibration jig provided by Intuitive Surgical.
2. Watch the **stereo_overlay** window: it shows the left frame tinted green and
   the right frame tinted magenta, superimposed.
3. Use the **← / →** arrow keys to shift ``horizontal_shift_px`` until the tint
   of the calibration cross cancels to neutral grey (the images fuse).
4. Use **↑ / ↓** to correct any vertical misalignment.
5. Use **``+`` / ``-``** (or ``o`` / ``p``) to adjust the crop size so the
   endoscope image fills the frame without black borders.

.. note::
   Proper camera alignment requires the calibration jig supplied by Intuitive
   Surgical.  The jig presents a known pattern through the endoscope optics so
   the left and right eye images can be precisely aligned.

**Saved as:** ``camera.alignment.horizontal_shift_px``,
``camera.alignment.vertical_shift_px``, ``camera.crop``.

Step 2 — Display Alignment (Depth Calibration)
-----------------------------------------------

**Goal:** find the ``display_horizontal_offset_px`` value that makes the stereo
display present content at the depth where the surgeon's hands typically operate.

The display offset is applied as an additional asymmetric crop folded into each
eye's ``videocrop`` element: the left eye's window shifts left and the right
eye's window shifts right by the same total amount, so the mixer stays a plain
side-by-side compositor with no ``xpos`` shifting and no black bars.

**How:**

1. Press **``g``** to enable the calibration grid and circle overlay.  The grid
   is drawn at a horizontal centre that shifts per eye by ``±display_offset / 2``.
2. Press **``[``** or **``]``** to decrease or increase the offset by 2 px until
   the coloured grid squares and the white circle are perfectly fused when
   viewing at the desired working depth.

**Saved as:** ``display_horizontal_offset_px``.

Color Calibration Tuning
--------------------------

Real-world endoscopes often have slight manufacturing or lighting differences
between the left and right lenses.  The ``c`` key cycles through automatic
color-matching modes:

1. **None** — Neutral pass-through (default).
2. **Left Ref** — Matches the right eye's brightness, contrast, and saturation
   to the left eye.
3. **Right Ref** — Matches the left eye to the right eye.
4. **Average** — Matches both eyes to their combined average.

Color correction is computed on the current frame and applied immediately to the
preview.  On exit the calibrated values are written to ``camera.left.color`` and
``camera.right.color`` in the JSON file.

.. note::
   Color calibration is applied in HSV space.  ``hue`` is always left at
   ``0.0`` by the automatic modes; it can be set manually in the JSON file if
   needed.
