# Trace

**Trace** is an open-source vehicle data logger built on the ESP32-S3. It collects, timestamps, and fuses data from multiple onboard sources — GNSS, IMU, OBD2 — and writes it to a CSV file on an SD card. A lightweight web interface served over Wi-Fi Access Point allows real-time monitoring and configuration without any dedicated app.

Trace was originally designed as a replacement for a commercial device that also drove an RGB LED strip. That LED subsystem is still present and functional, but the project's focus today is squarely on **data logging and analysis**.

---

## Features

### Data sources
- **GNSS** — position, speed, course, altitude, UTC time (ATGM336H via UART + TinyGPS++)
- **IMU** — longitudinal/lateral acceleration, roll, pitch, slope estimate with confidence (ISM330DHCX via I2C, Madgwick 6-DOF filter)
- **OBD2** — RPM, vehicle speed, engine load, throttle position, coolant temperature (Mode 01 PIDs via CAN/TWAI or UART simulator)

### Sensor fusion
- Heading estimated from gyroscope integration, corrected by GNSS course over ground (complementary filter with speed- and HDOP-weighted gain)
- Yaw rate smoothing via EMA
- Filtered lat/lon position
- Fused vehicle speed (GNSS primary, OBD2 fallback)

### Gear estimation
- Runtime gear detection based on RPM/speed ratio
- Per-gear calibration with hysteresis filtering
- Calibration data persisted to NVS (survives power cycles)
- Interactive calibration via serial commands

### Time synchronization
- Monotonic 64-bit microsecond clock (`esp_timer`)
- Soft UTC sync from GNSS fix, with EMA-filtered offset
- Per-sensor timestamps on every sample; age of each source logged per row
- Sync quality score (0–100) with decay after last valid fix

### Logging
- CSV output to SD card, one row per 500 ms
- Filename derived from GNSS UTC at acquisition start
- Automatic flush every 10 rows; graceful degradation if SD is absent
- Fields: position, GNSS quality, OBD2 data, IMU data, heading/yaw, gear estimate, monotonic and UTC timestamps, sensor ages, sync quality

### Web interface
- Wi-Fi Access Point, no infrastructure required
- REST API (`/api/status`, `/api/color`, `/api/mode`, `/api/params`)
- Mobile-first HTML UI served directly from the device

### RGB LED control *(legacy)*
- Retained for compatibility with the original hardware
- Modes: static color, fading, breathing, RPM-driven color gradient, RPM warning blink
- Configurable at runtime via web API or serial

---

## Hardware

| Component | Role |
|---|---|
| ESP32-S3 | Main MCU |
| ATGM336H | GNSS receiver (UART) |
| ISM330DHCX | 6-DOF IMU (I2C) |
| MCP2515 / TWAI | CAN bus interface for OBD2 |
| SD card (SPI) | Data storage |
| RGB LED (common anode) | Status / legacy feature |

Default pin assignments are defined as `#define` constants at the top of each manager header and can be overridden at compile time.

---

## Getting started

### Requirements
- [PlatformIO](https://platformio.org/) or Arduino IDE with ESP32 board support
- Libraries: `TinyGPSPlus`, `Preferences` (bundled with ESP32 core)

### Build & flash
```bash
# Clone the repo
git clone https://github.com/youruser/trace.git
cd trace

# Build (PlatformIO)
pio run -t upload
```

### OBD2 transport selection
By default the firmware compiles with the **UART simulator** transport (useful for development without a real vehicle). To target a real CAN bus, define `OBD2_USE_TWAI` in your build flags:

```ini
; platformio.ini
build_flags = -DOBD2_USE_TWAI
```

---

## Serial commands

Connect at **115200 baud**. All commands are terminated with `\n`.

### IMU calibration
| Command | Description |
|---|---|
| `cal_gyro` | Gyroscope bias calibration (vehicle stationary) |
| `cal_acc` | Accelerometer calibration (vehicle stationary and level) |
| `set_mounting <roll> <pitch>` | Set sensor mounting offset in degrees |
| `save_cal` | Save current calibration to NVS |
| `reset_cal` | Reset calibration to defaults |
| `show_cal` | Print current calibration parameters and time sync status |

### Gear calibration
| Command | Description |
|---|---|
| `gear_help` | List gear commands |
| `gear_show` | Show current gear calibration |
| `gear_cal <1..5>` | Start a capture window for the specified gear |
| `gear_capture` | Trigger a 2-second stable acquisition window |
| `gear_save` | Retrieve and confirm the last capture result |
| `gear_set <gear> <ratio>` | Manually set RPM/speed ratio for a gear |
| `gear_reset` | Clear all gear calibration data |

---

## CSV output format

Each row contains:

```
timestamp_utc_str, lat, lon, alt_m, sat, hdop,
speed_obd_kmh, acc_lon_G, acc_lat_G, roll_deg, pitch_deg,
slope_deg, slope_confidence, heading_deg, yawRate_dps, heading_confidence,
rpm, load_pct, throttle_pct, estimated_gear,
t_mono_us, utc_epoch_us, utc_valid, sync_quality,
imu_t_us, gnss_t_us, obd_speed_t_us,
imu_age_ms, gnss_age_ms, obd_speed_age_ms
```

The `*_age_ms` columns record how stale each sensor's data was at log time — useful for post-processing quality filtering.

---

## Project structure

```
├── main.cpp                  # Setup, loop, serial command handler
├── shared_data.h             # Global VehicleData struct (all producers/consumers)
├── gnss_manager.*            # GNSS UART + TinyGPS++ integration
├── imu_manager.*             # ISM330DHCX driver, Madgwick filter, calibration
├── obd2_manager.*            # OBD2 CAN decoder (TWAI + UART simulator)
├── vehicle_fusion_manager.*  # Sensor fusion: heading, position, speed
├── gear_estimator.*          # RPM/speed gear detection + NVS calibration
├── time_sync_manager.*       # Monotonic clock + GNSS soft UTC sync
├── sd_manager.*              # SPI SD card + CSV writer
├── web_server.*              # AP-mode WebServer + REST API
└── rgb_controller.*          # RGB LED driver (legacy)
```

---

## License

MIT
