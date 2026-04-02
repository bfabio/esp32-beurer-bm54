# No ZHA quirk needed.
#
# EP1 and EP2 use PressureMeasurement (0x0403); ZHA creates native pressure
# sensor entities for them automatically.
#
# EP3 uses AnalogInput (0x000C) for pulse rate and is intentionally ignored
# by ZHA (no entity factory for AnalogInput exists in ZHA).
#
# After pairing, change the unit of the two pressure entities from hPa to mmHg
# in the HA UI (Settings -> Entities -> entity -> Unit). HA will convert the
# stored hPa values (which are mmHg * 1.33322) back to the correct mmHg values.
