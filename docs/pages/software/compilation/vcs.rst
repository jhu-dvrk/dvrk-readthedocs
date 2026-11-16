.. _vcs:

************
Repositories
************

For ROS users, we recommend to use ``vcs`` to retrieve all the
repositories used by the dVRK software stack.  ``vcs`` allows users to
manage a set of repositories.  The source of all packages based on the
*cisst*/*SAW* libraries can be downloaded using ``vcs``.  Generic
compilation instructions and vcs files can be found on GitHub:
https://github.com/jhu-saw/vcs.

The following sections include the ``vcs`` command lines specific to
the dVRK for both ROS 1 and ROS 2.

The main dVRK manifests are provided for ROS 1 and ROS 2. Optional
components can have their own ROS or platform requirements.

.. note::
   The main repository for the dVRK software stack is
   |sawIntuitiveResearchKit|_.  This follows the naming convention used for
   all cisst/SAW components, i.e. ``<manufacturer><product>``. This is
   quite a long name and the acronym is not great: IRK. Hence the
   name dVRK.


Choosing a manifest
===================

Manifest names indicate the source dependencies they include:

* ``ros1-`` and ``ros2-`` distinguish repository sets or revisions specific
  to ROS 1 and ROS 2.
* ``ros-`` identifies a manifest shared by ROS 1 and ROS 2.
* No ROS prefix means that the manifest does not fetch cisst/ROS bridges or
  CRTK packages. These core component manifests can still be imported into
  a ROS workspace; the prefix does not describe the component's platform
  or runtime requirements.

Use the dVRK manifest matching your ROS version and development line, such
as ``ros2-dvrk-main.vcs`` or ``ros2-dvrk-devel.vcs``. A manifest's ``main``
or ``devel`` suffix selects its development line; individual repositories
can intentionally use other branches or release tags.

.. _vcs-add-components:

Adding components to an existing workspace
==========================================

Start with the main dVRK manifest, then import the manifest for any additional
SAW component into the same workspace's ``src`` directory. Repeated imports
add repositories to that workspace; there is no need to merge the YAML files
by hand.

Check that the component's shared dependencies are compatible with the dVRK
versions already checked out. Use matching ``main`` or ``devel`` manifests
where available. The ``--skip-existing`` option preserves existing checkouts,
including custom branches and release tags; it does not resolve incompatible
dependency requirements or update the workspace to another development line.

For example, to add Haply to an existing ROS 2 dVRK **devel** workspace:

.. code-block:: bash

   cd ~/ros2_ws/src
   vcs import --skip-existing --recursive --workers 1 --retry 3 \
     --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros-haply-devel.vcs
   cd ..
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash

For a dVRK main workspace, use ``ros-haply-main.vcs`` instead. Adjust the
workspace path to your installation. ROS 1 users follow the same import
procedure with a compatible manifest and rebuild using ``catkin build``.

Build dynamically loaded cisst/SAW components together with dVRK in one ROS 2
workspace, then source that workspace. This keeps the component and dVRK linked
to the same cisst/SAW libraries.

The following manifests are in the `jhu-saw/vcs repository
<https://github.com/jhu-saw/vcs>`_. Choose one matching your dVRK installation:

.. list-table:: Additional SAW components
   :header-rows: 1
   :widths: 26 37 37

   * - Component
     - main
     - devel
   * - ``sawHaplyService``
     - ``ros-haply-main.vcs``
     - ``ros-haply-devel.vcs``
   * - ``sawOpenXR``
     - ``openxr-main.vcs``
     - ``openxr-devel.vcs``
   * - ``sawForceDimensionSDK``
     - ``ros-force-dimension-main.vcs``
     - ``ros-force-dimension-devel.vcs``
   * - ``sawSensablePhantom``
     - ``ros-sensable-main.vcs``
     - ``ros-sensable-devel.vcs``
   * - ``sawOpenIGTLink``
     - ``open-igtlink-main.vcs``
     - ``open-igtlink-devel.vcs``
   * - ``sawSocketStreamer``
     - ``socket-streamer-main.vcs``
     - ``socket-streamer-devel.vcs``
   * - ``sawGenericInput``
     - ``generic-input-main.vcs``
     - ``generic-input-devel.vcs``

The dVRK devel manifests already include ``sawGenericInput``. Its standalone
manifest also adds it to workspaces initialized from other dVRK manifests.
Core component manifests do not fetch the full dVRK stack or every optional
ROS frontend dependency. Install system libraries and device runtimes using
the component's own setup instructions.
