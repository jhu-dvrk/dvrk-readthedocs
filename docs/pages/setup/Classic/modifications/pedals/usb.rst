.. _pedals-usb:

USB foot pedals
===============

Use `sawGenericInput <https://github.com/jhu-saw/sawGenericInput>`_ with
``mtsEvdev`` to connect USB foot pedals on Linux. It supports pedals exposed
as keyboard keys or joystick buttons through Linux input event devices. The device must provide a ``/dev/input/event*`` node; a generic HID interface alone is not sufficient.

Compilation
-----------

Add ``sawGenericInput`` using :ref:`Adding components to an existing workspace
<vcs-add-components>`, with ``generic-input-main.vcs`` or
``generic-input-devel.vcs`` to match the dVRK installation. The dVRK devel
manifests already include this repository.

Install the component's libevdev development dependency, then rebuild and
source the dVRK workspace:

.. code-block:: bash

   sudo apt install libevdev-dev
   cd ~/ros2_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash

The input library is the single CMake package ``sawGenericInputCore``.
To build the optional Qt configurator and monitor as well:

.. code-block:: bash

   colcon build --packages-up-to sawGenericInputCore \
     --cmake-args -DsawGenericInput_WITH_QT=ON
   source install/setup.bash

Device selection and permissions
--------------------------------

Use ``sawGenericInputConfigQt`` to select the pedal input node, observe its
``KEY_*`` or ``BTN_*`` events, and save a configuration. A device can be
selected by name and USB vendor/product IDs, or by a stable path such as
``/dev/input/by-id/...-event-kbd``. Event numbers can change when devices are
reconnected; a ``/dev/input/js*`` path is not used by ``mtsEvdev``.

For PCsensor pedals with USB ID ``3553:b001``, install the supplied udev rule
from the workspace root:

.. code-block:: bash

   sudo install -m 0644 \
     src/cisst-saw/sawGenericInput/core/share/udev/70-saw-evdev-pcsensor.rules \
     /etc/udev/rules.d/
   sudo udevadm control --reload-rules

Unplug and reconnect the pedals afterward. The rule grants the active local
seat user access to that device. Other pedal models need a rule matching
their own USB identifiers; see the component README for permission options.

Configuration
-------------

Follow :ref:`USB foot pedal configuration <config-pedals-usb>` to connect the
pedal events to clutch, camera, and operator presence. For the PCsensor
example, test the mappings with:

.. code-block:: bash

   sawGenericInputQtExample \
     -j src/cisst-saw/sawGenericInput/core/share/pcsensor-pedals.json

Only run one monitor or dVRK input component against the pedals at a time.
The example uses exclusive capture, so pedal keys do not reach desktop
applications while it owns the device.
