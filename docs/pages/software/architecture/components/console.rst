.. _console_component:

Console
#######



Selecting teleoperation pairs
*****************************

On a clinical da Vinci system, there are usually 3 PSMs and only 2
MTMs.  So the user has to select which PSMs should be teleoperated at
a given time (selected) and which one should be left alone
(unselected).  The clinical system uses a menu (or buttons) on the
console to set the configuration (e.g. MTMR will drive PSM1 and PSM3
while MTML will drive PSM2).  Then the operator can use a foot pedal
to toggle the PSMs on the MTM configured to drive 2 PSMs.  For the da
Vinci Classic and S, the operator had to do a "quick tap" on the
clutch pedal.

In practice, for the dVRK, you can swap using the “Clutch” foot pedal
if there are two PSMs controlled by a single MTM.  This is done using
a “quick tap”, i.e. about 1/10 of a second tap on the clutch. This is
similar to the Intuitive implementation on the clinical first two
generations of daVincis.

To do a full swap, say from MTMR/PSM1 & MTML/PSM2 to MTMR/PSM2 &
MTML/PSM1 you will need to use ROS topics (see :ref:`console
API<api-console>`).

We use the ``diagnostic_msgs::KeyValue`` which is a pair of strings to
represent the MTM/PSM pair.

For example, if a user with two MTMs (MTML and MTMR) and two PSMs
(PSM1 and PSM2) wants to swap the two PSMs while the application is
running, they first have to declare all 4 possible combinations in the
console JSON configuration file.  Then:

* In your case, you would start with:
  
  Status: **MTMR/PSM1 - MTML/PSM2**
  
* Then use the topic ``/dvrk/console/teleop/select_teleop_psm`` with ``MTMR/""`` (use empty string to free the MTMR)
  
  Status:  **MTMR/"" - MTML/PSM2**
  
* Then assign PSM2 to MTMR: ``/dvrk/console/teleop/select_teleop_psm MTMR/PSM2``
  
  Status: **MTMR/PSM2 - MTML/""**
  
* Finally assign PSM1 to MTML: ``/dvrk/console/teleop/select_teleop_psm MTML/PSM1``
  
  Status: **MTMR/PSM2 - MTML/PSM1**

While you’re changing the selected pairs, you should make sure your requests are valid and listen to the ROS topics:

* ``/dvrk/console/teleop/teleop_psm_selected``
* ``/dvrk/console/teleop/teleop_psm_unselected``
