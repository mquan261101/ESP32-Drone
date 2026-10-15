# ESP32 Reconnaissance Quadcopter: Real-Time GPS Telemetry & FPV AI Integration

An open-source, low-cost reconnaissance quadcopter platform developed on the dual-core **ESP32** microcontroller. This repository contains the complete firmware, embedded web server, hardware pinouts, and Python Ground Control Station (GCS) pipeline for **250Hz real-time flight control**, **GNSS GPS/BDS live telemetry mapping**, and **low-latency FPV video streaming with YOLOv8 object detection**.

Developed as a Capstone Graduation Thesis at **Ho Chi Minh City University of Technology (HCMUT)**.

---

## 📌 Key System Features

### 1. Flight Controller Firmware
* **High-Frequency Control Loop**: Runs at a strict **250 Hz (4 ms)** interval with loop overrun detection.
* **1D Kalman Filter**: Fuses MPU-6050 accelerometer and gyroscope data for smooth, drift-free Roll/Pitch estimation.
* **2D Kalman Filter**: Fuses BMP280 barometer static pressure and vertical inertial acceleration for altitude and vertical velocity ($v_z$) estimation.
* **Cascade PID Control**: Multi-tiered PID loops covering Rate Control, Angle Mode, and Altitude Hold (Velocity Vertical).
* **Motor Mixing & Safety**: Quad-X PWM power distribution (LEDC), Integral Anti-Windup, low-throttle cut-off, and SBUS failsafe handling.

### 2. GNSS GPS Telemetry & Embedded Web GCS
* **GNSS Data Parsing**: Hardware UART1 (`GPIO32/33` @ 9600 bps) decoding NMEA-0183 ASCII streams (`$GPGGA`, `$GPRMC`) via `TinyGPS++` with XOR checksum validation.
* **Trajectory Memory Buffer**: Maintains a static 200-point FIFO history array (`latHistory`/`lonHistory`) in SRAM.
* **Software Watchdog**: Automatically flags `hasFix = false` if no valid NMEA sentence is received within 5 seconds (5000 ms).
* **Embedded mDNS Web Server**: Broadcasts domain `http://drone.local` on port 80, serving RESTful `/data` JSON APIs.
* **Web Mapping Interface**: Interactive HTML/JS GCS built with **Leaflet.js** and **OpenStreetMap**, polling JSON at 1s intervals to render live red trajectory polylines, ground speed, MSL altitude, course, and satellite tracking (15–23 satellites locked).

### 3. FPV Video Stream & YOLOv8 AI Pipeline (ESP32-CAM & Python GCS)
* **Independent SoftAP Wi-Fi**: ESP32-CAM hosts an autonomous Wi-Fi Access Point (`ESP32CAM`) for off-grid field operations.
* **HTTP JPEG Streaming**: Low-latency video stream (150–250 ms) at HD/SVGA resolution using OV2640.
* **AI Computer Vision GCS**: Python GCS script ingests HTTP video frames into **YOLOv8 Nano** (`yolov8n.pt`) for real-time target recognition, bounding box plotting, and label classification (25–30 FPS).

---

## 🏗️ Hardware Architecture & Bill of Materials (BOM)

| Component | Specification / Model | Quantity | Interface / Protocol |
| :--- | :--- | :---: | :--- |
| **Microcontroller (FC)** | ESP32 NodeMCU-32S (Tensilica Xtensa LX6 @ 240MHz) | 1 | I2C, UART, PWM (LEDC) |
| **IMU Sensor** | MPU-6050 (6-DOF Gyroscope & Accelerometer) | 1 | I2C (`0x68`) @ 400kHz |
| **Barometer Sensor** | BMP280 (Atmospheric Pressure & Altitude) | 1 | I2C (`0x76`) @ 400kHz |
| **GNSS GPS Module** | MKE-M19 GP-02 (GPS/BDS Dual-Mode) | 1 | Hardware UART1 (9600 bps) |
| **FPV Camera Module** | ESP32-CAM (OV2640 2MP + PSRAM) | 1 | Wi-Fi 802.11 b/g/n SoftAP |
| **Airframe** | F450 Quadcopter Frame (450mm wheelbase) | 1 | Integrated PDB Board |
| **BLDC Motors** | A2212 1000KV Brushless Motors | 4 | 3-Phase Direct Drive |
| **Propellers** | 8045 Propellers (2 CW, 2 CCW) | 2 Pairs | Top Prop Nut Fastened |
| **Electronic Speed Controls** | 30A ESCs (5V BEC output) | 4 | PWM 250Hz input |
| **RC Transmitter / Receiver** | Microzone MC6C (TX) + MC7RE (RX) | 1 Set | 2.4GHz FHSS, SBUS protocol |
| **Battery** | LiPo 3S 2200mAh 30C | 1 | Deans / XT60 Connector |
| **Power Isolation** | Independent 5V/3A BEC for ESP32-CAM | 1 | Voltage Regulation |

---

## 🔌 ESP32 Hardware Pinout Mapping

### Flight Controller (ESP32 NodeMCU-32S)
```
          +--------------------------------------+
          |           ESP32 NodeMCU-32S          |
          |                                      |
   PWM M1 | GPIO27                        GPIO21 | I2C SDA (MPU6050/BMP280)
   PWM M2 | GPIO14                        GPIO22 | I2C SCL (MPU6050/BMP280)
   PWM M3 | GPIO12                        GPIO16 | SBUS RX (Serial2 RX)
   PWM M4 | GPIO13                        GPIO02 | Status LED
   GPS RX | GPIO32 (UART1 RX)                    |
   GPS TX | GPIO33 (UART1 TX)              GND   | Common Ground
          | 5V (from BEC)                 3.3V   | Sensor VCC
          +--------------------------------------+
```

---

## 📁 Repository Directory Layout

```
├── firmware/
│   ├── FlightController/
│   │   ├── FlightController.ino       # Core 1 Flight Control & PID loop (250Hz)
│   │   ├── sbus.h                     # SBUS receiver protocol parser
│   │   └── BasicLinearAlgebra.h       # Matrix math for 2D Kalman filter
│   ├── GPSTelemetry/
│   │   ├── GPSTelemetry.ino           # Core 0 GPS parsing & Web Server
│   │   └── index.html                 # Embedded Leaflet.js Ground Station GCS
│   └── ESP32CAM_FPV/
│       ├── ESP32CAM_FPV.ino           # Wi-Fi SoftAP & HTTP JPEG Streamer
│       └── camera_pins.h              # AI-Thinker OV2640 pin definitions
├── gcs_ai/
│   ├── detect_yolo.py                 # Python OpenCV + YOLOv8 real-time detector
│   └── requirements.txt               # Dependencies for AI Ground Station
├── docs/
│   └── capstone_report.pdf            # Academic thesis documentation
└── README.md                          # Project documentation
```

---

## 🚀 Getting Started

### Prerequisites & Tools
* **Arduino IDE** (v2.0+) or **PlatformIO**.
* **ESP32 Board Package**: Install via Board Manager (`esp32` by Espressif Systems v2.0+).
* **Python**: v3.8+ (for AI Ground Control Station).

### Required Arduino Libraries
Install the following libraries via Arduino Library Manager:
* `TinyGPS++` by Mikal Hart
* `BasicLinearAlgebra` by Tom Stewart
* `bolderflight/sbus` by Bolder Flight Systems

---

## 📥 Installation & Setup Instructions

### 1. Flashing Firmware to ESP32 Flight Controller
1. Open `firmware/FlightController/FlightController.ino`.
2. Connect ESP32 NodeMCU-32S via USB (CP2102 / CH340 driver).
3. Select Board: `ESP32 Dev Module`, Flash Frequency: `80MHz`, CPU Frequency: `240MHz`.
4. Upload code and verify `Serial Monitor` at **115200 bps** prints `Ready to fly!`.

### 2. Setting Up GPS Telemetry & Web GCS
1. Open `firmware/GPSTelemetry/GPSTelemetry.ino`.
2. Configure your local Wi-Fi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Upload to the GPS ESP32 module.
4. Once connected, open any browser on the same network and navigate to:
   `http://drone.local` or `http://<ESP32_IP_ADDRESS>`

### 3. Setting Up ESP32-CAM & Python YOLOv8 Detector
1. Upload `firmware/ESP32CAM_FPV/ESP32CAM_FPV.ino` to the ESP32-CAM module using an FTDI Programmer (Set Jumper to `GPIO0` -> `GND` for flashing).
2. Connect your ground computer to the Wi-Fi AP generated by the drone:
   * **SSID**: `ESP32CAM`
3. Launch the Python YOLOv8 detection pipeline:
   ```bash
   cd gcs_ai
   pip install -r requirements.txt
   python detect_yolo.py
   ```

---

