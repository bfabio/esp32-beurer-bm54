# ESP32-H2 BLE-Zigbee bridge for Beurer BM54

Bridges a Beurer BM54 Bluetooth blood pressure monitor into Home Assistant
via Zigbee (ZHA). The ESP32-H2 acts as a BLE central to the BM54 and a
Zigbee end device to the coordinator.

```
BM54 --[BLE]--> ESP32-H2 --[Zigbee]--> ZHA coordinator --> Home Assistant
```

## Hardware

- ESP32-H2-mini-1 (BLE 5.3 + 802.15.4)
- WS2812 LED on GPIO8

## How it works

The BM54 stores measurements in memory. When the user presses the Bluetooth
button on the device, the ESP32 connects via BLE, receives all stored
readings, and forwards the most recent one to the Zigbee coordinator.

Systolic and diastolic pressure use the ZCL PressureMeasurement cluster.
Pulse rate uses AnalogInput.

## Build

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32h2/get-started/) v5.1.4+.

```bash
. /path/to/esp-idf/export.sh
idf.py set-target esp32h2
idf.py menuconfig  # set BM54 MAC address under "Beurer BM54 Bridge"
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## First pairing

1. Flash and boot the ESP32. LED blinks blue (Zigbee steering).
2. Open ZHA in Home Assistant and add a new device.
3. Once joined, LED turns solid green.
4. Press the Bluetooth button on the BM54. The ESP32 connects and bonds.
5. Enter the 6-digit passkey shown on the BM54 screen via the serial monitor.
6. Measurements appear as pressure sensor entities in HA.

On subsequent connections the stored bond is reused (no passkey needed).

## Factory reset

After boot, press and hold the BOOT button (GPIO9). The LED blinks red with
increasing speed. After 3 seconds it flashes red-white-green to confirm.
This erases both BLE bonds and Zigbee network data.

## LED states

| Color | Pattern | Meaning |
|-------|---------|---------|
| Blue | slow blink | Zigbee steering / connecting |
| Green | solid | ready, listening for BM54 |
| Green | fast blink | BLE data transfer |
| Red | fast blink | error |
