.. _usage-tips:

****
Tips
****

Terminal
########

* You might want to try ``terminator`` instead of the default Ubuntu terminal.
  To install it, ``sudo apt install terminator``. In terminator, you can split
  your window vertically and horizontally as much as you want
  (https://gnome-terminator.readthedocs.io).  You probably should change the
  scrolling setting to infinite
  (`documentation <https://gnome-terminator.readthedocs.io/en/latest/preferences.html#scrolling>`_).

* Linux keeps a history of your commands. You can use the **Up** and **Down**
  arrows to go through your history and retrieve past commands. The command
  ``history`` will show you the full history.

* Linux shell offers auto-completion. Hit the **Tab** key and, it will either
  complete your command or present options. **This is extremely useful.** For
  example, you can just type ``dvr`` followed by 2 **Tab** and you will see all
  the commands starting with ``dvr``. Most applications provided along the dVRK
  start with ``dvr``, ``qla`` or ``saw``. Type these prefixes followed by
  **Tab** and, you'll get access to most dVRK related commands.

* Auto-completion works with ROS. For example, type ``ros2`` followed by a
  **space** and **Tab**, the different options will be shown.  If you type
  ``ros2 run dvr`` followed by **Tab**, all the ROS dVRK packages will be
  listed. After ``ros2 run dvrk_robot``, **Tab** will list all the possible
  nodes in the ``dvrk_robot`` package.

* If you are unsure about a command spelling, option or even a ROS topic, hit
  **Tab**. It will likely help.

* Linux terminal uses shortcuts similar to Emacs.  *ctrl+a* goes to beginning of
  line, *ctrl+e* goes to end of line, *ctrl+arrow* left/right goes from word to
  word, *ctrl+k* deletes all characters after your cursor.

* ROS commands tend to mess the terminal. You can type ``reset`` in the terminal
  to reset to the default settings.

dVRK
####

* Many cisst/SAW applications, including the dVRK ones, will create log files.
  You will quickly find many ``cisstLog.txt`` files in your working directories.
  If you are comfortable with the ``find`` command, you can do ``find . -name
  "cisstLog*.txt" -exec rm {} \'`` to find and delete them. This is not
  necessarily easy to remember so, we provide a small :ref:`script to clean all
  the cisst log files<remove-logs>` as well as some backup files generated
  during the configuration and calibration steps. Go to the top directory you
  want to clean and type: ``dvrk-remove-logs.py``.

* When you run into issues with the dVRK, always scroll up to find the initial
  error message. The last message might just be the consequence of previous
  errors and doesn't tell anything about the actual issue.

* The log file, ``cisstLog-<date>.txt`` contains all the messages you can see in
  the dVRK GUI and in the terminal. It is better so send this file along
  but reports than sending screenshots. The log has a lot more information too,
  such as the full path of every configuration file used and their content.

* When running a dVRK application, you might want to reset the terminal and
  clean the logs first.  For example, use: ``reset; dvrk-remove-logs.py -f; ros2
  run dvrk_robot dvrk_system -j <your_system.json>``. This way, you can scroll
  up in the terminal to see the first messages displayed and the only cisstLog
  file will correspond to your last run.