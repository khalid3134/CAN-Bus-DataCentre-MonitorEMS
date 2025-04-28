# CAN Bus Data Centre Monitor  
Author: Khalid Suliman

## Overview
This project monitors temperature, humidity, and pressure using an Arduino CAN Bus network.  
It controls fans and vent actuators with manual and automatic control based on indoor conditions. It detects sensor failures, high temperatures, actuator mismatches, and CAN Bus health issues.

**Main Features:**
- Touchscreen GUI for manual and auto control.
- Real-time CAN Bus communication.
- Automatic fan/vent control with adjustable thresholds.
- Actuator feedback monitoring (vent).
- Sensor error detection and alert system.
- Persistent EEPROM storage for settings.
- Full system state Google Sheets uploads via (commented for now).

---


Hardware Datasheets available in `/Hardware_Docs`.

---


## Tests and Expected Outputs

| Test | Action | Expected Behaviour |
|:-----|:-------|:--------------------|
| **DHT22 unconnected** | Unplug DHT22 for 60s | **Red Alert:** "DHT22 sensor not updating" |
| **BMP280 unconnected** | Unplug BMP280 for 60s | **Red Alert:** "BMP280 sensor not updating" |
| **Reconnected sensor** | Reconnect sensor properly | **Yellow Alert:** "DHT22 had no update" or "BMP280 had no update" |
| **Indoor overheat** | Heat DHT22 above threshold | **Red Alert:** "Inside Temp too high!" |
| **Outdoor overheat** | Heat BMP280 above threshold | **Red Alert:** "Outside Temp too high!" |
| **Sender node disconnect** | Power off sender node | **Red Alert:** "CAN ISSUE, SENDER OR GUI NOT ON BUS" |
| **Sender node reconnect** | Power on sender node again | **Yellow Alert:** "SENDER OR GUI WAS DISCONNECTED" |
| **Actuator mismatch** | Toggle OPEN and unplug feedback pin | **Red Alert:** "VENT mismatch (no response)" |
| **Fix actuator** | Restore actuator to correct position | **Yellow Alert:** "Previous Vent mismatch" |
| **Manual fan/vent toggle** | Toggle buttons on screen | Immediate toggle and sync |
| **Auto Mode Enable** | Press Auto Mode toggle | Fan and Vent auto-control based on thresholds |

---

## EEPROM Saved Settings
- Fan state
- Vent state
- Auto mode
- Auto Temp threshold
- Auto Humidity threshold

All settings are restored automatically on reboot.

---

## WiFi Uploads
- WiFi upload to Google Sheets (disabled by default, easily enabled by uncommenting in code).
- Data uploading every 15 minutes if WiFi is connected.

---

# End of README