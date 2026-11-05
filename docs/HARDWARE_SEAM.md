# Hardware Seam Architecture & Integration Guide

The Thermal SAR Ground Control Station (GCS) enforces a strict **Hardware Seam**:
the user interface, detection engine, tracking algorithm, and flight recorder never interact with or import simulator modules. All operations communicate exclusively through the abstract `DroneLink` interface in `src/gcs/link/base.py`.

```
           +-------------------------------------------------------+
           |     GCS Core Pipeline (UI, Detector, Tracker, Rec)    |
           +-------------------------------------------------------+
                                      |
                               [ DroneLink API ]
                                      |
         +----------------------------+----------------------------+
         |                            |                            |
+-----------------+          +-----------------+          +-----------------+
|     SimLink     |          |   MavlinkLink   |          |    Esp32Link    |
| (Current Phase) |          | (Autopilot)     |          | (Thermal & GPS) |
+-----------------+          +-----------------+          +-----------------+
         |                            |                            |
  [ Python Sim ]             [ Pixhawk/ArduPilot ]        [ MLX90640 + ESP32 ]
```

---

## 1. Abstract DroneLink Interface Contract

Located in [`src/gcs/link/base.py`](file:///d:/AIR%20Support/src/gcs/link/base.py):

- **Lifecycle:**
  - `connect_link() -> bool`: Initializes connection and starts worker streams.
  - `disconnect_link() -> None`: Gracefully halts streams and closes sockets/serial ports.
  - `is_connected() -> bool`: Returns telemetry link status.
- **Outbound Command:**
  - `send_control(ControlInput) -> None`: Dispatches operator inputs (sticks, switches, RTL).
- **Qt Inbound Signals:**
  - `telemetry_received(Telemetry)`: Emitted at 10 Hz (pose, attitude, battery, GPS fix).
  - `thermal_received(ThermalFrame)`: Emitted at 8 Hz (calibrated 24x32 float32 array in °C with pose snapshot).
  - `video_received(VideoFrame)`: Emitted at 15–20 Hz (640x480 RGB visual stream).
  - `link_status_changed(str)`: Emits status messages ("Connected", "Signal Degraded", etc.).

---

## 2. Integrating a Real Autopilot (`MavlinkLink`)

To interface with an ArduPilot, PX4, or Pixhawk flight controller:

### Step 1: Install Dependencies
```bash
pip install pymavlink pyserial
```

### Step 2: Implement Connection & Background Thread
In `src/gcs/link/mavlink_link.py`:
1. Use `pymavlink.mavutil.mavlink_connection(connection_str, baud=57600)`:
   - For USB radio (SiK 915MHz/433MHz): `connection_str="COMx"` or `"/dev/ttyUSB0"`.
   - For Wi-Fi / Companion computer: `connection_str="udpin:0.0.0.0:14550"`.
2. Spawn a background `QThread` or `threading.Thread` with a non-blocking `recv_match(blocking=False)`.

### Step 3: Message Mapping to `Telemetry`
Map incoming MAVLink packets directly to `gcs.core.models.Telemetry`:
- `GLOBAL_POSITION_INT`:
  - `lat = msg.lat / 1e7`
  - `lon = msg.lon / 1e7`
  - `alt_msl = msg.alt / 1000.0`
  - `alt_agl = msg.relative_alt / 1000.0`
  - `ground_speed = hypot(msg.vx, msg.vy) / 100.0`
  - `vertical_speed = -msg.vz / 100.0`
  - `heading_deg = msg.hdg / 100.0`
- `SYS_STATUS`:
  - `battery_v = msg.voltage_battery / 1000.0`
  - `battery_pct = msg.battery_remaining`
- `GPS_RAW_INT`:
  - `gps_fix = "3D" if msg.fix_type >= 3 else ("2D" if msg.fix_type == 2 else "NO_FIX")`
  - `gps_sats = msg.satellites_visible`
- `HEARTBEAT`:
  - `mode = mavutil.mode_string_v10(msg)`
  - `armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)`

### Step 4: Dispatching Control Commands
In `send_control(self, control: ControlInput)`:
- Send `mav.manual_control_send(target_system, x, y, z, r, buttons)`.
- Or send guided velocity commands via `SET_POSITION_TARGET_LOCAL_NED`.

---

## 3. Integrating the Real Payload (`Esp32Link`)

The real search-and-rescue payload combines an **MLX90640 32x24 Far-Infrared sensor**, an **ESP32-S3 microcontroller**, and a **u-blox GPS module**.

### Step 1: ESP32 Firmware Packet Structure
The ESP32 samples the MLX90640 via I2C at 8 Hz and packs the telemetry and temperatures into a compact binary frame over UDP or USB-UART (921600 baud):

| Offset (Bytes) | Type | Field | Description |
|---|---|---|---|
| 0 .. 1 | uint16 | `0x55AA` | Synchronization magic preamble |
| 2 .. 5 | uint32 | `timestamp_ms` | Payload uptime clock |
| 6 .. 9 | float32 | `latitude` | GPS latitude from u-blox |
| 10 .. 13 | float32 | `longitude` | GPS longitude from u-blox |
| 14 .. 17 | float32 | `altitude_agl` | Lidar/barometric AGL (meters) |
| 18 .. 21 | float32 | `heading_deg` | Electronic compass azimuth |
| 22 .. 1557 | 768 x float16 | `temperatures` | 32 x 24 pixel temperatures in °C |
| 1558 .. 1559 | uint16 | `crc16` | Checksum verification |

### Step 2: Implementation in `Esp32Link`
In `src/gcs/link/esp32_link.py`:
1. Listen on UDP port 5005 (or read binary stream from `COMx` at 921600 baud).
2. Scan for preamble `0x55AA`, verify `crc16`.
3. Unpack float16 values into a `(24, 32)` float32 NumPy array.
4. Construct `DronePose(lat, lon, alt_agl, heading_deg)`.
5. Construct `ThermalFrame(timestamp, data, sensor_fov_deg=(55.0, 35.0), drone_pose=pose)`.
6. Emit `self.thermal_received.emit(thermal_frame)`.

---

## 4. Swapping Links in `app.py`

In `src/gcs/app.py`, link selection is a single configuration switch:

```python
# Simulation mode:
link = SimLink(config=app_config)

# Real hardware mode (when hardware is connected):
# link = MavlinkLink(connection_str="udpin:0.0.0.0:14550")
# payload_link = Esp32Link(udp_port=5005)
```
No UI widgets, detector filters, or storage files need to be modified.
