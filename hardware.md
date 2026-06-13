# 🤖 Savelij — Hardware Reference

> **Platform:** Yahboom ROSMASTER X3 (Superior Edition)  
> **Compute:** NVIDIA Jetson Orin NX 8GB  
> **Purpose:** Autonomous cat patrol robot — SLAM, Nav2, YOLOv8 visual detection, voice interaction  
> **Operating environment:** Indoor, 1–5m range, domestic rooms  
> **ROS version:** ROS2 Humble  

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Power Flow Diagram](#2-power-flow-diagram)
3. [USB Hub Board — YB-ERV02-V2.0](#3-usb-hub-board--yb-erv02-v20)
4. [Expansion Board — YB-ERF01-V3.0](#4-expansion-board--yb-erf01-v30)
5. [Compute — Jetson Orin NX 8GB](#5-compute--jetson-orin-nx-8gb)
6. [Depth Camera — Orbbec Astra FHD](#6-depth-camera--orbbec-astra-fhd)
7. [LiDAR — YDLIDAR X3 Pro](#7-lidar--ydlidar-x3-pro)
8. [Voice Module — YB-MASR-V1.0](#8-voice-module--yb-masr-v10)
9. [Port Assignment Master Table](#9-port-assignment-master-table)
10. [ROS2 Topic Map](#10-ros2-topic-map)
11. [Feature-to-Hardware Matrix](#11-feature-to-hardware-matrix)
12. [Known Limitations & Gotchas](#12-known-limitations--gotchas)
13. [Links & Resources](#13-links--resources)

---

## 1. System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SAVELIJ BOT SYSTEM                           │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │               JETSON ORIN NX 8GB  (192.168.0.120)           │   │
│  │   ROS2 Humble · SLAM · Nav2 · YOLOv8 · All high-level logic │   │
│  └───────┬──────────────────────────────────────────────────────┘   │
│          │ USB-A ×5                                                  │
│          ├── [L] 7-inch display                                      │
│          ├── [C] Orbbec Astra FHD  (depth camera)                   │
│          ├── [C] Orbbec Astra FHD  (pair port)                      │
│          ├── [R] Expansion board   (via Micro USB)                   │
│          └── [R] YDLIDAR X3 Pro    (via USB-serial adapter)          │
│                                                                     │
│  ┌──────────────────┐       ┌──────────────────────────────────┐    │
│  │  USB Hub Board   │       │       Expansion Board            │    │
│  │  YB-ERV02-V2.0   │──12V──│       YB-ERF01-V3.0              │    │
│  │                  │       │  Motors · Servos · IMU · CAN     │    │
│  │  IN:  12V (×2)  │       │  I2C · RGB · SBUS · SWD          │    │
│  │       5V micro   │       └──────────────────────────────────┘    │
│  │  OUT: USB ×4    │                                                │
│  │       VBUS 5V   │       ┌──────────────────────────────────┐    │
│  └──────────────────┘       │     Voice Module YB-MASR-V1.0   │    │
│          ▲                  │  CI1302 ASR · STC8H · Speaker   │    │
│          │                  │  UART or I2C to Jetson/Exp.Bd   │    │
│   3S2P 18650 Battery        └──────────────────────────────────┘    │
│   (Panasonic NCR18650B)                                             │
│   ~12.6V fully charged                                              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Power Flow Diagram

```
  ┌─────────────────────────────────┐
  │  3S2P 18650 Battery Pack        │
  │  Panasonic NCR18650B cells      │
  │  JYS-6042-3S-TK BMS             │
  │  ~12.6V full · ~11.1V nominal   │
  │  ~6800mAh effective capacity    │
  └────────────┬────────────────────┘
               │ 12V DC
               ▼
  ┌─────────────────────────────────┐
  │  USB Hub Board  YB-ERV02-V2.0   │
  │                                 │
  │  ┌──────────┐  ┌─────────────┐  │
  │  │ ON/OFF   │  │ DC 12V ×2   │  │◄── Battery
  │  │ Switch   │  │ DCIN 6V-24V │  │
  │  └──────────┘  └─────────────┘  │
  │                                 │
  │  VBUS 5V ──────────────────────►│──► Jetson (5V rail if needed)
  │  USB1-USB4 ────────────────────►│──► Peripheral devices
  └────────────┬────────────────────┘
               │ 12V DC
               ▼
  ┌─────────────────────────────────┐
  │  Expansion Board  YB-ERF01-V3.0 │
  │                                 │
  │  12V IN → Motor H-bridge        │──► Motors M1–M4
  │  Type-C 5V OUT ────────────────►│──► Voice module / sensors
  │  DC 5V OUT ────────────────────►│──► Logic peripherals
  └─────────────────────────────────┘

  Jetson Orin NX powers itself via:
  └── USB-C 5V or dedicated power connector from hub VBUS
```

---

## 3. USB Hub Board — YB-ERV02-V2.0

**Role:** Power distribution spine and USB data routing hub.

**Product:** Yahboom YB-ERV02-V2.0  
**Location in bot:** Center of chassis, below the Jetson mount  

### Interfaces

| Direction | Label | Connector | Notes |
|-----------|-------|-----------|-------|
| **IN** | USB data input | White USB-A | Data path from Jetson |
| **IN** | DC 12V input (×2) | Screw terminal `DCIN 6V-24V` | Both are parallel — wire battery here |
| **IN** | DC 5V input | Micro USB `DCIN 5V-2A` | Auxiliary 5V if needed |
| **OUT** | USB data output (×2) | White USB-A (top) | Data out to peripherals |
| **OUT** | USB1–USB4 | Right-side USB-A ports | Powered USB for peripherals |
| **OUT** | VBUS 5V | Header `VBUS` | 5V bus power rail |

### Controls

| Component | Function |
|-----------|----------|
| ON/OFF slide switch | Cuts all board outputs. Use for safe power cycling |

### Key ICs (visible on PCB)

| Component | Purpose |
|-----------|---------|
| Large MCU (center) | USB hub controller IC |
| Blue electrolytic cap | Bulk power filtering (absorbs inrush) |
| Red indicator LEDs | Power and data status |

### Notes

- The two DC 12V screw terminals are **electrically parallel** — wire battery to either, or both for redundancy
- Data path is strictly: **Jetson → USB IN → USB data outputs → lidar / camera**. The USB ports are NOT an independent hub; they pass through from the Jetson
- When flipping the ON/OFF switch, allow 2–3 seconds for capacitors to discharge before reconnecting
- If the Jetson draws too much current through VBUS (e.g. when external display is attached), consider a direct barrel jack supply to the Jetson

---

## 4. Expansion Board — YB-ERF01-V3.0

**Role:** Low-level actuator control, sensor buses, secondary power regulation. The STM32-based bridge between Jetson ROS2 and physical hardware.

**Product:** Yahboom YB-ERF01-V3.0  
**Communicates with Jetson via:** Micro USB serial (appears as `/dev/ttyUSBx`)

### Power Interfaces

| Label | Connector | Direction | Notes |
|-------|-----------|-----------|-------|
| DC 12V input | Large terminal block (top-right) | IN | From USB hub board |
| DC 12V output | Same block | OUT | Passthrough / daisy-chain |
| Type-C DC | Type-C port | OUT | 5V regulated output |
| DC 5V output | 2-pin header | OUT | Logic-level 5V rail |

### Communication Interfaces

| Interface | Connector | Notes |
|-----------|-----------|-------|
| Micro USB data | Micro USB port | Serial to Jetson — main comms channel |
| I2C | Header: `GND SDA SCL 3.3V` | For sensors, OLED displays, etc. |
| CAN bus | Header: `CAN` | Future peripherals, arm actuators |
| SWD debug | Header: `SWD` (top-left) | STM32 firmware update via ST-Link |

### Actuator Interfaces

| Interface | Pins | Notes |
|-----------|------|-------|
| Motor M1–M4 | `3U3 · GND · H_A · H_B` per motor | H-bridge outputs. Match to wheel position carefully |
| PWM servo | Standard 3-pin servo header | RC-style servos, pan/tilt |
| Serial servo (robotic arm) | Bus header | Smart digital servos (e.g. Dynamixel-compatible) |
| SBUS receiver | `SBUS` header | RC model aircraft remote input |

### Motor Wiring Convention

```
Expansion Board            Wheel mapping (top view)
                           ┌──────────────┐
M1 ──► Front-Left          │  M1       M2 │
M2 ──► Front-Right         │              │
M3 ──► Rear-Left           │  M3       M4 │
M4 ──► Rear-Right          └──────────────┘

H_A / H_B pin polarity determines forward/reverse per motor.
Verify by running each motor independently before SLAM.
```

### Onboard Sensors

| Sensor | Type | Interface | Notes |
|--------|------|-----------|-------|
| IMU (onboard) | 6-axis inertial | Internal MCU bus | Feeds odometry data; complements wheel encoders |
| RGB light bar | Addressable LEDs | `RGB` header | Status indicator — configurable per ROS2 state |

### Firmware

The expansion board runs a Yahboom-provided STM32 firmware that:
- Reads motor encoder ticks → publishes odometry
- Drives motor H-bridge from `/cmd_vel` or internal serial protocol
- Reads IMU → publishes IMU data
- Controls RGB, servos, buzzer via serial commands

Flash new firmware via SWD (ST-Link) if you need custom low-level behavior.

---

## 5. Compute — Jetson Orin NX 8GB

**Role:** All ROS2 high-level computation — SLAM, Nav2 path planning, YOLOv8 cat detection, voice processing, WebSocket comms.

**Specs:** NVIDIA Jetson Orin NX 8GB, JetPack 5.x, Ubuntu 20.04  
**IP on local network:** `192.168.0.120`  
**SSH:** `ssh bots@192.168.0.120`

### Physical Interface Layout

```
  TOP EDGE
  ┌──────────────────────────────────────────────────────┐
  │  [ANT1]                                    [ANT2]    │  ← WiFi antenna U.FL connectors
  │                    ┌─────────┐                       │
  │                    │ COOLING │    [FAN JST]           │  ← cooling fan connector
  │   [CAM1 MIPI]      │  FAN   │    [OLED I2C]  ████   │  ← color-coded header
  │                    └─────────┘                       │
  │   [SSD M.2 — on BACK of board]                       │
  └──────────────────────────────────────────────────────┘
  BOTTOM EDGE — USB-A ports (left to right):
  ┌────────────────────────────────────────────────────┐
  │  [USB-A] ──────── 7-inch touch display             │
  │  [USB-A] [USB-A] ─ Orbbec Astra FHD (depth cam)   │
  │  [USB-A] [USB-A] ─ Expansion board + YDLIDAR X3   │
  └────────────────────────────────────────────────────┘
```

### USB Port Assignments (fixed)

| Port | Device | Cable |
|------|--------|-------|
| USB-A leftmost | 7-inch HDMI/USB display | USB-A to USB-A |
| USB-A center pair | Orbbec Astra FHD | USB-A (single cable, power+data) |
| USB-A rightmost pair (port 1) | Expansion board YB-ERF01 | USB-A to Micro USB |
| USB-A rightmost pair (port 2) | YDLIDAR X3 Pro | USB-A to Type-C (adapter board) |

> ⚠️ All 5 USB-A ports are occupied in full configuration. Voice module must use expansion board UART or I2C — not a direct USB connection to the Jetson.

### Other Connectors

| Connector | Location | Assignment |
|-----------|----------|------------|
| WiFi antenna ×2 | Top edge | U.FL → external antenna wires |
| OLED I2C | Right edge, color-coded header | Status OLED display (0x3C typical) |
| Cooling fan | Right edge, JST | PWM-controlled fan |
| CAM1 MIPI CSI | Left edge | Currently unused — available for CSI camera |
| SSD M.2 NVMe | Back of board | System SSD installed |

### Performance Notes

| Workload | Expected behavior |
|----------|------------------|
| ROS2 + SLAM | Moderate CPU load, GPU idle |
| YOLOv8 inference | GPU active, ~30fps on 640px input |
| SLAM + Nav2 + YOLO simultaneously | Monitor thermal throttle — fan must be functional |
| WebSocket streaming | CPU load depends on resolution; use JPEG compression |

---

## 6. Depth Camera — Orbbec Astra FHD

**Role:** 3D spatial perception — obstacle avoidance costmap, RTAB-Map depth input, RGB feed for YOLOv8 cat detection, object distance estimation.

**Technology:** Structured light (850nm IR projector + IR camera + custom Orbbec ASIC)  
**Product page:** https://www.orbbec.com/products/structured-light-camera/astra-series/  
**ROS2 driver:** https://github.com/orbbec/OrbbecSDK_ROS2

### Specifications

| Parameter | Value |
|-----------|-------|
| Depth technology | Structured light |
| Depth range | 0.6 m – 8 m |
| Depth resolution | 640 × 480 @ 30 fps |
| Depth FOV | 58.4° H × 45.5° V |
| Depth accuracy | ±3mm @ 1m (typical) |
| RGB resolution | **1920 × 1080 @ 30 fps** (FHD) |
| RGB FOV | ~66.1° H × 40.2° V |
| Interface | USB 2.0 Type-A — single cable (power + data) |
| Power draw | Average < 2.4 W |
| IMU | ❌ Not present |
| Active cooling | ✅ Yes (fan for stable long-run) |
| Operating temp | 10°C – 40°C, 10–85% RH |
| D2C alignment | ✅ Supported |
| Data outputs | Depth map · RGB · IR · Point Cloud |

### How Structured Light Works

```
  IR Projector         IR Camera
       │                  │
       │  Known pattern   │  Captured deformed pattern
       │ ─────────────►   │
       │                  ▼
       │          ┌──────────────┐
       │          │ Orbbec ASIC  │
       │          │ depth solver │
       │          └──────┬───────┘
       │                 │
       ▼                 ▼
   Scene             Depth map (per pixel distance)
```

The IR projector casts a structured dot pattern. Deformation of that pattern captured by the IR camera reveals depth at every pixel. On-chip ASIC handles all computation — no CPU load for depth processing.

**Implication:** The camera produces depth data independently; the Jetson only handles the streams.

### ROS2 Integration

**Driver setup (run once on Jetson):**
```bash
cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select orbbec_camera
source install/setup.bash
```

**Launch:**
```bash
ros2 launch orbbec_camera astra_pro.launch.py
```

**Key topics published:**

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | Full 1080p RGB |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | 640×480 depth map |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 30 Hz | 3D point cloud |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Intrinsics |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Intrinsics |

> ⚠️ **QoS:** Default is `sensor_data` (Best Effort). Set RViz2 reliability to Best Effort to avoid topic subscription failures.

**Enable D2C alignment** (for cat detection — maps depth onto color frame):
```bash
# In launch file or via parameter:
ros2 param set /camera/orbbec_camera_node depth_registration true
```

### Relevant Use Cases for Savelij

| Feature | Camera contribution |
|---------|-------------------|
| SLAM (RTAB-Map) | Depth stream → 3D voxel map reconstruction |
| Nav2 obstacle avoidance | Depth → `depthimage_to_laserscan` → costmap |
| YOLOv8 cat detection | RGB 1080p → detection input |
| Per-cat identification | RGB crop → classification model |
| Cat distance estimation | Depth map lookup at bounding box center |
| Visual odometry | RGB + depth → RTAB-Map VO fallback |

### Blind Zone Warning

```
   0m    0.6m                  8m
   │──────│──────────────────────│
   ████   ·  valid depth range   ·
   blind
   zone
```

Objects closer than **0.6m** produce no depth data. Cats sitting right under the camera are invisible to it. Combine with YDLIDAR X3 Pro for full close-range coverage.

---

## 7. LiDAR — YDLIDAR X3 Pro

**Role:** 360° 2D plane scanning for SLAM map building, Nav2 costmap obstacle layer, wall-following patrol, room boundary detection.

**Manufacturer:** EAI Technology (YDLIDAR brand)  
**Price:** $79  
**Product page:** https://category.yahboom.net/products/eai-x3  
**Yahboom tutorials:** https://www.yahboom.com/study/YDLIDAR-X3  
**Yahboom GitHub:** https://github.com/YahboomTechnology/EAI-X3-X3ProLidar  
**ROS2 driver:** https://github.com/YDLIDAR/ydlidar_ros2_driver (branch: `humble`)

### Technology: Laser Triangulation

```
  Rotating mirror
       │
  Laser ──► scene ──► reflected back ──► CCD sensor
                                              │
                          angle of spot on CCD → distance
```

A laser beam is directed at the scene. The reflected spot lands on a CCD at a position determined by the target's distance. Depth is computed from the **angle** of the reflected ray — no time measurement needed. This makes the sensor cheap, accurate at short–medium range, but noisier beyond ~5m.

### Specifications

| Parameter | X3 | **X3 Pro (Savelij)** |
|-----------|-----|---------------------|
| Ranging technology | Laser triangulation | Laser triangulation |
| Scan FOV | 360° | **360°** |
| Max range | 8 m | **8 m** |
| Min range (blind zone) | ~0.1 m | **~0.1 m** |
| Sampling frequency | 3000 samples/s | **4000 samples/s** |
| Scan frequency | 5–10 Hz | **5–10 Hz (auto-adjustable)** |
| Serial baud rate | 115200 | **115200** |
| Interface | USB serial adapter | **USB serial adapter** |
| Laser class | Class I (eye-safe) | **Class I** |

### Angular Resolution (computed)

```
  Angular resolution = scan_freq (Hz) / sample_rate (Hz) × 360°

  @ 5 Hz  / 4000 samples → 0.45°/point  (finest, slowest)
  @ 7 Hz  / 4000 samples → 0.63°/point  (recommended default)
  @ 10 Hz / 4000 samples → 0.90°/point  (fastest, coarsest)
```

For domestic-scale patrol (3–6m rooms), 7 Hz gives a good balance — ~570 points per scan, updating 7 times per second.

### USB Adapter Board

The X3 Pro connects via a small adapter board with **two Type-C ports**:

| Port | Label | Purpose |
|------|-------|---------|
| Left | `USB_DATA` | **Data + power** — use this first |
| Right | `USB_PWR` | Auxiliary power only (if motor stalls) |

Connect `USB_DATA` → Type-C to USB-A adapter → Jetson rightmost USB-A pair (second port).

### ROS2 Integration

**Install YDLidar SDK first:**
```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig
```

**Install driver (Humble branch):**
```bash
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git \
  ~/ros2_ws/src/ydlidar_ros2_driver
cd ~/ros2_ws && colcon build --packages-select ydlidar_ros2_driver
source install/setup.bash
```

**udev rule (persistent device name):**
```bash
# Add to /etc/udev/rules.d/99-ydlidar.rules:
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  SYMLINK+="ydlidar", MODE="0666"

sudo udevadm control --reload-rules && sudo udevadm trigger
```

**X3 Pro params** (`~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml`):
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ydlidar
    baudrate: 115200          # X3/X3 Pro: 115200. NOT 230400 (that's X4/G4)
    lidar_type: 1             # 1 = triangulation type
    device_type: 0
    isSingleChannel: false
    intensity: false
    sample_rate: 4            # 4 = 4000Hz (X3 Pro). Use 3 for basic X3
    frequency: 7.0            # scan Hz — tune 5–10
    reversion: true
    inverted: false
    resolution_fixed: true
    auto_reconnect: true
    angle_min: -180.0
    angle_max: 180.0
    range_min: 0.1
    range_max: 8.0
    frame_id: laser_frame
    invalid_range_is_inf: false
```

**Launch:**
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**Verify:**
```bash
ros2 topic hz /scan                        # expect ~7 Hz
ros2 topic echo /scan | head -40           # inspect ranges array
ros2 run rviz2 rviz2                       # add LaserScan, topic: /scan
```

**Key topics:**

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/scan` | `sensor_msgs/LaserScan` | 5–10 Hz | 360° range array |

### TF Configuration

The static transform `base_link → laser_frame` must match the physical mounting position. Measure offsets from robot center to LiDAR center:

```bash
# In launch file or as a separate node:
ros2 run tf2_ros static_transform_publisher \
  x y z roll pitch yaw base_link laser_frame
# Example (LiDAR centered, 15cm above base):
ros2 run tf2_ros static_transform_publisher \
  0 0 0.15 0 0 0 base_link laser_frame
```

### Relevant Use Cases for Savelij

| Feature | LiDAR contribution |
|---------|-------------------|
| Gmapping / Cartographer SLAM | `/scan` → 2D occupancy grid |
| Nav2 local costmap | `/scan` → obstacle inflation layer |
| Room boundary detection | 360° sweep identifies walls, doors, furniture perimeter |
| Close-range obstacle avoidance | Reads to ~10cm — covers depth camera blind zone |
| Wall-following patrol | Consistent offset from scan returns in front arc |
| Return-to-dock | Known scan signature near dock position |

---

## 8. Voice Module — YB-MASR-V1.0

**Role:** Offline voice command recognition and audio feedback. Wake-word triggered, custom command words, no cloud dependency.

**SKU:** 6000400536  
**Price:** $19  
**Product page:** https://category.yahboom.net/products/voice_module_asr-tts  
**Tutorials:** https://www.yahboom.net/study/Voice-interaction

### Physical Layout

```
  FRONT                          BACK (PCB side)
  ┌────────────────┐             ┌────────────────────────────┐
  │                │             │  [Type-C]  ← power / flash │
  │   ●●●●●●       │             │                            │
  │  ●●●●●●●       │             │  [Amplifier chip]          │
  │   ●●●●●●       │  Speaker    │  [CI1302 ASR chip]         │
  │  ●●●●●●●       │             │  [STC8H MCU chip]          │
  │   ●●●●●●       │             │  [Anti-reverse protection] │
  │                │             │                            │
  │  [Slide SW]    │             │  [IIC header]              │
  │  [Reset BTN]   │             │  [Serial header]           │
  │  [MCU LED]     │             │                            │
  │  [PWR LED]     │             └────────────────────────────┘
  │                │
  │     (MIC)      │  ← high-performance microphone
  └────────────────┘

  BOTTOM — two PH2.0 connectors:
  ┌──────────────────────────────────────┐
  │  5V · GND · TX · RX                 │  ← UART to host
  │  RX1 · TX1 · GND · 5V              │  ← secondary UART
  │  GND · SDA · SCL · 3.3V            │  ← I2C to host
  └──────────────────────────────────────┘
```

### Key ICs

| Chip | Role |
|------|------|
| **CI1302** | Main ASR (Automatic Speech Recognition) engine. Contains a neural network processor with deep learning noise reduction and echo cancellation |
| **STC8H** | MCU bridge — converts CI1302 recognition events into serial UART frames or I2C messages the host reads |
| **Amplifier** | Drives the onboard speaker for TTS / audio feedback |

### Capabilities

| Feature | Detail |
|---------|--------|
| Preset commands | 110+ factory-loaded voice commands |
| Custom commands | Via Yahboom web tool → generate firmware → flash via PC over Type-C |
| Storage | 2MB internal, ~120 command words max |
| Wake word | Customizable |
| Languages | Chinese + English |
| Recognition rate | 99% at ≤5m range |
| Communication mode | UART **or** I2C — choose one at integration time |

### Cables Included

| Cable | Length | Purpose |
|-------|--------|---------|
| PH2.0-4Pin to DuPont | 20cm | Connect to Jetson GPIO UART or expansion board |
| PH2.0-4Pin to PH2.0-4Pin | 20cm | Board-to-board |
| Type-C | 100cm | Power + firmware flashing |
| M3×5mm screws ×2 | — | Mounting |
| M3×6+4mm copper standoffs ×2 | — | Mounting |

### Integration with Savelij

**Connection path (recommended):** Voice module UART TX/RX → Expansion board serial header → Jetson reads via `/dev/ttyUSBx` serial

**Alternative:** PH2.0 DuPont cable → Jetson GPIO UART pins (if expansion board serial port is occupied)

**ROS2 node concept:**
```
/dev/ttyUSB1  →  voice_serial_node  →  /voice_cmd (String)
                                              │
                                      voice_dispatcher_node
                                              │
                          ┌───────────────────┼──────────────┐
                          ▼                   ▼              ▼
                    /cmd_vel           /patrol/start    /camera/snapshot
                  (move robot)       (begin patrol)    (take photo)
```

**Custom command word examples for Savelij:**

| Command word | Action |
|-------------|--------|
| "Savelij, go forward" | Publish to `/cmd_vel` |
| "Savelij, stop" | Zero velocity |
| "Savelij, start patrol" | Trigger patrol scheduler node |
| "Savelij, find cat" | Enable YOLOv8 detection mode |
| "Savelij, go home" | Trigger Nav2 return-to-dock |
| "Savelij, take picture" | Trigger camera snapshot |

**I2C address note:** If using I2C mode, verify voice module I2C address does not conflict with OLED (typically 0x3C). Scan bus: `i2cdetect -y 1`

### Firmware Customization Workflow

```
1. Go to Yahboom web customization tool
2. Enter custom command words (Chinese or English)
3. Download generated .bin firmware file
4. Connect module via Type-C to PC
5. Flash using Yahboom PC tool (Windows)
6. Module reboots with new vocabulary
```

---

## 9. Port Assignment Master Table

Complete wiring reference — every physical connection in the system.

| From | To | Cable | Notes |
|------|----|-------|-------|
| Battery (+/−) | USB hub board DCIN 6V-24V | 14AWG silicone wire | Both screw terminals in parallel |
| USB hub board 12V OUT | Expansion board 12V IN | DC cable | Powers motors |
| USB hub board USB_DATA | Jetson USB-A (center pair, via hub) | USB-A to USB-A | Data routing |
| Expansion board Micro USB | Jetson USB-A rightmost (port 1) | Micro USB to USB-A | Serial comms, `/dev/ttyUSB0` |
| YDLIDAR X3 Pro USB_DATA | Jetson USB-A rightmost (port 2) | Type-C to USB-A | Serial comms, `/dev/ydlidar` |
| Orbbec Astra FHD | Jetson USB-A center pair | USB-A (single cable) | Power + depth + RGB data |
| 7-inch display | Jetson USB-A leftmost | USB-A to USB-A | Touch + USB data |
| Voice module UART | Expansion board serial header | PH2.0-4Pin to DuPont | `5V GND TX RX` |
| OLED display | Jetson OLED-I2C header | JST/I2C cable | Address 0x3C |
| WiFi antennas ×2 | Jetson top U.FL | U.FL cables | Both required for 2×2 MIMO |
| Cooling fan | Jetson fan JST connector | JST cable | PWM-controlled |
| SSD (NVMe) | Jetson M.2 back slot | — | Already installed |

---

## 10. ROS2 Topic Map

All topics Savelij produces and consumes in normal operation.

### Sensor Inputs (produced by hardware nodes)

| Topic | Type | Source node | Rate |
|-------|------|-------------|------|
| `/scan` | `LaserScan` | `ydlidar_ros2_driver_node` | 5–10 Hz |
| `/camera/depth/image_raw` | `Image` | `orbbec_camera_node` | 30 Hz |
| `/camera/color/image_raw` | `Image` | `orbbec_camera_node` | 30 Hz |
| `/camera/depth/points` | `PointCloud2` | `orbbec_camera_node` | 30 Hz |
| `/imu/data` | `Imu` | expansion board serial node | ~100 Hz |
| `/odom` | `Odometry` | expansion board serial node | ~50 Hz |
| `/voice_cmd` | `String` | voice serial node | on event |

### Command Outputs (consumed by hardware nodes)

| Topic | Type | Consumer | Effect |
|-------|------|----------|--------|
| `/cmd_vel` | `Twist` | expansion board node | Motor velocity |
| `/rgb_control` | `String` | expansion board node | RGB light bar color |
| `/servo_cmd` | `Int32` | expansion board node | Servo position |

### Computed / Navigation Topics

| Topic | Type | Producer | Consumer |
|-------|------|----------|---------|
| `/map` | `OccupancyGrid` | SLAM node | Nav2, RViz2 |
| `/tf` | `TFMessage` | multiple | everyone |
| `/goal_pose` | `PoseStamped` | operator / patrol node | Nav2 |
| `/detection/cats` | `Detection2DArray` | YOLOv8 node | logger, alerter |
| `/costmap/local` | `OccupancyGrid` | Nav2 | path planner |

---

## 11. Feature-to-Hardware Matrix

How each planned Savelij feature maps to specific hardware.

| Feature | LiDAR | Depth Cam | IMU | Motors | Voice | Jetson GPU |
|---------|:-----:|:---------:|:---:|:------:|:-----:|:----------:|
| 2D SLAM mapping | ✅ primary | optional | ✅ assist | — | — | — |
| 3D SLAM (RTAB-Map) | ✅ assist | ✅ primary | ✅ assist | — | — | — |
| Nav2 navigation | ✅ obstacle | ✅ obstacle | — | ✅ drive | — | — |
| Wall-following patrol | ✅ primary | — | — | ✅ drive | — | — |
| Cat detection (YOLO) | — | ✅ RGB | — | — | — | ✅ inference |
| Cat distance estimate | — | ✅ depth | — | — | — | — |
| Per-cat identity | — | ✅ RGB | — | — | — | ✅ classify |
| Image capture | — | ✅ RGB | — | — | — | — |
| Voice command control | — | — | — | ✅ execute | ✅ primary | — |
| Audio feedback (TTS) | — | — | — | — | ✅ speaker | — |
| Return-to-dock | ✅ landmark | — | — | ✅ drive | — | — |
| Odometry fallback | — | — | ✅ primary | ✅ encoders | — | — |
| Close obstacle avoidance | ✅ primary | ✅ depth | — | ✅ react | — | — |

---

## 12. Known Limitations & Gotchas

### Power

- **USB current budget:** Jetson USB-A ports supply 5V/0.9A each. With 5 devices (display, camera ×2, expansion, lidar), total draw is close to limit. Monitor for USB resets under load (`dmesg | grep usb`)
- **Battery voltage sag:** Under full motor load, battery voltage can drop 0.5–1V. This can trigger BMS low-voltage cutoff — ensure motors don't stall under load for extended periods

### LiDAR (YDLIDAR X3 Pro)

- **Baud rate is 115200** — not 230400. Using wrong baud produces garbage data with no error message
- **Glass and mirrors** produce phantom returns or missing data — the patrol map near fish tanks or mirrors will be inaccurate
- **Scan plane is fixed height** — cats lying flat on floor may be below or in the scan plane depending on mounting height. Mount LiDAR at ~15–20cm for best coverage
- **USB_DATA port usually sufficient** — only add USB_PWR if motor stalls or scan data stutters

### Depth Camera (Orbbec Astra FHD)

- **0.6m blind zone** — nothing closer than 60cm produces depth. Cats directly under the camera are invisible to depth
- **Sunlight degrades structured light** — IR pattern washes out near bright windows. Schedule patrol away from high-sun periods if needed
- **USB 2.0 bandwidth ceiling** — running 640×480 depth + 1080p RGB simultaneously is near the practical USB 2.0 limit. If frame drops occur, reduce RGB resolution or disable one stream
- **No IMU** — unlike Astra 2, this model has no built-in IMU. Odometry comes from expansion board IMU + encoders only

### Voice Module

- **Windows-only firmware flash tool** — custom vocabulary requires a Windows machine for the official flash utility. Keep a Windows VM or partition available
- **I2C address conflict risk** — if using I2C mode, scan for conflicts with OLED (`i2cdetect -y 1`) before connecting
- **5m recognition limit** — at larger room distances, false triggers or missed words increase. Design patrol waypoints to bring robot closer to operator for voice interaction

### Expansion Board

- **Motor polarity** — H_A / H_B wiring determines forward/reverse per motor. Test each motor individually with a simple `/cmd_vel` command before SLAM. Miswired motors will spin the wrong way and destroy map quality
- **IMU calibration** — the onboard IMU needs calibration (static + dynamic) before use in odometry. Run the Yahboom calibration routine after assembly
- **SWD flashing requires ST-Link** — if custom firmware needed, you'll need an ST-Link V2 (~$5) connected to the SWD header

### Jetson

- **All USB-A ports occupied** — no spare Jetson USB ports in full configuration. Any additional USB devices must go through the hub board's USB1–USB4 ports
- **OLED I2C vs. other I2C devices** — OLED typically at address 0x3C. If adding more I2C sensors, verify addresses don't conflict: `i2cdetect -y 0` and `i2cdetect -y 1`
- **Thermal throttling** — running SLAM + Nav2 + YOLOv8 simultaneously under heavy GPU load can trigger thermal throttle. Ensure fan is spinning and check: `tegrastats`

---

## 13. Links & Resources

### Official Hardware Docs

| Component | Link |
|-----------|------|
| Yahboom ROSMASTER X3 | https://category.yahboom.net/products/rosmaster-x3 |
| USB Hub board tutorials | https://www.yahboom.com/study/ROSMASTER-X3 |
| Voice module product | https://category.yahboom.net/products/voice_module_asr-tts |
| Voice module tutorials | https://www.yahboom.net/study/Voice-interaction |
| YDLIDAR X3 Pro product | https://category.yahboom.net/products/eai-x3 |
| YDLIDAR X3 tutorials | https://www.yahboom.com/study/YDLIDAR-X3 |
| YDLIDAR GitHub (Yahboom) | https://github.com/YahboomTechnology/EAI-X3-X3ProLidar |
| Orbbec Astra series | https://www.orbbec.com/products/structured-light-camera/astra-series/ |

### ROS2 Drivers & SDKs

| Component | Driver | Branch |
|-----------|--------|--------|
| Orbbec depth camera | https://github.com/orbbec/OrbbecSDK_ROS2 | `v2-main` |
| YDLIDAR X3 Pro | https://github.com/YDLIDAR/ydlidar_ros2_driver | `humble` |
| YDLidar SDK (dependency) | https://github.com/YDLIDAR/YDLidar-SDK | `master` |
| Legacy Orbbec ROS2 | https://github.com/orbbec/ros2_astra_camera | `main` |
| Voice module ROS example | https://www.yahboom.net/study/Voice-interaction | — |

### Development & Tooling

| Tool | Purpose |
|------|---------|
| `tegrastats` | Jetson GPU/CPU/thermal monitor |
| `ros2 topic hz /scan` | Verify LiDAR publish rate |
| `ros2 topic hz /camera/depth/image_raw` | Verify camera publish rate |
| `i2cdetect -y 1` | Scan I2C bus for address conflicts |
| `dmesg \| grep usb` | Diagnose USB enumeration / resets |
| `ros2 run rviz2 rviz2` | Visual debugging of all sensor data |
| `ros2 run rqt_graph rqt_graph` | Visualize node/topic graph |

### Key ROS2 Packages (install on Jetson)

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rtabmap-ros \
  ros-humble-depthimage-to-laserscan \
  ros-humble-tf2-ros \
  ros-humble-robot-localization \
  ros-humble-cartographer-ros
```

---
