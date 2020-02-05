BLE Protocol

Write to INDEX_CURRENT to read a reading.

Wait for READING to be notified.

If you don't hear a notification then write to INDEX_CURRENT again until you do or until you timeout.

Once you get the notification read the READING characteristic and confirm that its index is the same as the index that you requested.

DO NOT queue writes. Only write INDEX_CURRENT once and wait for the READING to be read before continuing requesting a new one.

NOTE that READING will ONLY be updated if you write the INDEX_CURRENT. It will be stable until you write again. Even if the underlying index on the board gets updated with a new value. That value will not be sent over BLE until you request the same index again.

Right now the main unique identifier for a reading is the timestamp. If there is a power failure it is possible for their to be timestamps that are duplicated. This is sufficiently rare that we can ignore this case. It is possible that a handful of readings are lost and that is honestly fine.
