Introduction
============

The real da Vinci system uses a head sensor to detect if the operator
is present.  Without the operator, the system will not enable the
tele-operation.  For the dVRK, we have used a foot pedal as a dead man
switch to detect if the operator is present (usually the "COAG" foot
pedal).  This is a reasonable solution for brief experiments but it's
not very convenient.  In this page we describe how to either create a
"dVRK" head sensor from cheap parts or hack the existing da Vinci head
sensor.  The later option is much nicer but it requires a full da
Vinci system and it's important to note that you will need to keep the
plastic covers on the surgeon's console (or find an alternative way to
mount the head sensor).

.. note::

   Some HMDs (head mounted displays) also provide a way to detect if
   the user has their head in the console.  We found a nice HMD that
   provides both a better stereo view than the ISI HRSV and a
   replacement for the head sensor (see :ref:`Goovis <goovis>`).
