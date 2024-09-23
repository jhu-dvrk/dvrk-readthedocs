******************
Bluetooth (SUJ Si)
******************

.. warning::

   This section is specific to the dESSJ used for the Si SUJ
   
You first need to make sure you have a Bluetooth adapter on your PC
and said adapter is turned on.  You can use GUI tools for this or the
command line tool ``hciconfig``.  For the Si SUJ, you will need to
place your Bluetooth adapter really close to the SUJ so we recommend
using a USB3 extension cable and a USB Bluetooth dongle.

To test the dESSJ communication itself, you can use the
``bluetoothctl`` command line tool.  These tools are also useful to
find the MAC address of each dESSJ board.  We recommend to power the
controllers one by one so you can clearly identify with dESSJ BLE MAC
address correspond to which arm.  Then write down each MAC address and
stick a label with the last 4 characters of the MAC address on each
SUJ arm.

``hciconfig``
=============

* ``hciconfig`` list all Bluetooth adapters.  In the examples below,
  ``hci0`` identifies the Bluetooth adapter found
* ``sudo hciconfig hci0 down`` to stop Bluetooth
* ``sudo hciconfig hci0 up`` to start Bluetooth

``bluetoothctl``
================

Command line examples
---------------------

* ``bluetoothctl list`` to figure out if the computer has a Bluetooth
  adapter
* ``bluetoothctl devices`` to list devices found.  For Si SUJ,
  ``bluetoothctl devices| grep -i suj``.  Then note the MAC Address
  (Media Access Control Address) for the Arduino board on the dESSJ
* ``bluetoothctl info 68:7E:57:32:FD:631`` to get info.  UUID info is
  defined in ino (Arduino program)
* ``bluetoothctl connect 68:7E:57:32:FD:63``

Interactive shell
-----------------

Start ``bluetoothctl``.  When using ``bluetoothctl`` shell, note that
auto-complete works very well.

* ``devices`` to list available BT devices and find UUID
* ``connect 68:7E:57:32:FD:63``, the MAC address will differ (but
  auto-complete helps)
* ``info 68:7E:57:32:FD:63``, not required but will provide more data
  re. the Arduino BLE
* ``menu gatt`` to get access to attributes
* ``list-attributes`` to see all attributes
* ``select-attribute
  /org/bluez/hci0/dev_68_7E_57_32_FD_63/service000a/char000b``, the
  service name will differ based on the MAC address
* ``read`` should return the binary result as well as the human
  readable string.  You can repeat this command multiple times or you
  can use ``notify on`` and ``notify off`` to get new values
  automatically
* ``back`` to get back to previous menu
* ``disconnect 68:7E:57:32:FD:63`` to disconnect
