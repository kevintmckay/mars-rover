# Sawppy Shopping List

## Summary

| Category | Est. Cost |
|----------|-----------|
| Servos & Control | $160 |
| **Autonomy Sensors** | **$459** |
| **Autonomy Electronics** | **$20** |
| **Wiring & Cables** | **$39** |
| Electronics (misc) | $8 |
| Power (misc) | $22 |
| Aluminum Extrusions | $15-45 |
| Shafts & Bearings | $48 |
| Hardware (inserts, brass screws, sleeve) | $30 |
| 3D Printing Filament | $100 |
| **BASE TOTAL** | **$383-413** |
| **AUTONOMY TOTAL** | **$901-931** |

*Note: Excludes items already owned (Pi, LiPo, microSD, JST, Deans, M3 hardware)*

---

## 1. Servos & Control (Amazon/Hiwonder)

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| 2 | LX-16A Bundle (5 servos + BusLinker) | $75 | $150 | [Amazon](https://amzn.to/2WAm4Wl) |
| - | *OR* 10x LX-16A individually | $15 ea | $150 | [Amazon](https://amzn.to/2z0VVqV) |
| 1 | BusLinker/Debug Board (if not bundled) | $10 | $10 | [Amazon](https://amzn.to/2T4V8vZ) |

**Subtotal: ~$160**

---

## 2. Autonomy Sensors (Amazon/Slamtec/Intel)

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| 1 | RPLidar A1 (12m 2D Lidar) | $99 | $99 | [Amazon](https://www.amazon.com/Stemedu-Distance-Developed-Omnidirectional-Scanning/dp/B07L89TT6F) |
| 1 | Intel RealSense D435i (Depth Camera + IMU) | $300 | $300 | [Amazon](https://www.amazon.com/Intel-RealSense-Depth-Camera-D435i/dp/B07MWR2YJB) |
| 1 | BNO055 IMU (9-axis) | $35 | $35 | [Adafruit](https://www.adafruit.com/product/2472) |
| 1 | USB 3.0 Powered Hub (4-port) | $25 | $25 | [Amazon](https://www.amazon.com/atolla-Charging-Splitter-Individual-Switches/dp/B083XTKV8V) |

**Subtotal: ~$459**

*Note: D435i includes built-in IMU, but external BNO055 provides better accuracy for EKF fusion*

---

## 3. Autonomy Electronics (Amazon)

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| 1 | DC-DC Buck 5V/5A USB-C (Pi 5 power) | $12 | $12 | [Amazon](https://www.amazon.com/PlusRoc-Waterproof-Converter-Compatible-Raspberry/dp/B09DGDQ48H) |
| 1 | INA219 Battery Monitor (optional) | $8 | $8 | [Amazon](https://www.amazon.com/HiLetgo-INA219-Bi-directional-Current-Breakout/dp/B01ICN5OAM) |

**Subtotal: ~$20**

*Note: Removed USB-to-Serial CH340 (RPLidar A1 has built-in USB) and 5V/3A buck (servos run on battery voltage, not 5V)*

---

## 4. Wiring & Cables (Amazon)

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| 1 | 14AWG Silicone Wire (red+black, 3m each) | $10 | $10 | [Amazon](https://www.amazon.com/BNTECHGO-Silicone-Flexible-Strands-Stranded/dp/B01ABOPMEI) |
| 1 | XT30 Connector Set (5 pairs) | $6 | $6 | [Amazon](https://www.amazon.com/Amass-Connector-Female-Bullet-Battery/dp/B074PN6N8V) |
| 10 | JST 2.0 3P Extension Cables (500mm) | $1.20 | $12 | [Amazon](https://www.amazon.com/Micro-Connector-Female-Cable-200mm/dp/B012HJ89TK) |
| 1 | USB-C PD Cable (for Pi 5 power input) | $8 | $8 | [Amazon](https://www.amazon.com/Anker-Powerline-Delivery-Samsung-MacBook/dp/B088FK68TR) |
| 1 | I2C Logic Level Shifter (3.3V↔5V) | $3 | $3 | [Amazon](https://www.amazon.com/KeeYees-Channels-Converter-Bi-Directional-Shifter/dp/B07LG646VS) |

**Subtotal: ~$39**

### Power Wiring Diagram

```
2S LiPo (7.4V, 5200mAh)
    │
    ├─[XT30]─[5A Fuse]──→ Servo Bus (direct 7.4V to all 10 servos)
    │                         └── BusLinker ──USB──→ Pi 5 (USB 2.0)
    │
    └─[XT30]─[5A Fuse]──→ 5V/5A Buck ──USB-C──→ Pi 5 Power
                               │
                               └──→ USB Hub ──→ RealSense D435i
                                          └──→ RPLidar A1
```

**Important:** LX-16A servos require 6-8.4V (2S LiPo range). The servo bus cables carry both power and signal. The BusLinker provides USB-serial conversion but does NOT provide servo power - that comes directly from the battery through the daisy-chain bus.

---

## 5. Raspberry Pi & Electronics (Amazon) - Base Build

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| ~~1~~ | ~~Raspberry Pi 5 (8GB/16GB)~~ | - | - | *OWNED* |
| ~~1~~ | ~~microSD Card 32GB+~~ | - | - | *OWNED* |
| 1 | 5V Regulator (MP1584EN) | $8 | $8 | [Amazon](https://amzn.to/2AyxH7T) |
| ~~1~~ | ~~JST 2.0 3P Connectors (servo cables)~~ | - | - | *OWNED* |

**Subtotal: ~$8**

---

## 6. Power System (Amazon)

| Qty | Item | Unit Price | Total | Link |
|-----|------|------------|-------|------|
| ~~1~~ | ~~2S LiPo Battery 5200mAh~~ | - | - | *OWNED* |
| ~~1~~ | ~~Deans T-Plug Connectors~~ | - | - | *OWNED* |
| 1 | Inline Fuse Holder + 5A Fuses (x2) | $10 | $10 | [Amazon](https://amzn.to/3cCqfXf) |
| ~~1~~ | ~~LiPo Battery Charger~~ | - | - | *OWNED* |
| 1 | Power Switch (SPST, 10A rated) | $6 | $6 | [Amazon](https://amzn.to/2ApRB4L) |
| 1 | Voltage LED Display | $6 | $6 | [Amazon](https://amzn.to/364aeqD) |

**Subtotal: ~$22**

---

## 7. Aluminum Extrusions (Misumi)

Order from: [Misumi HFS3 15x15mm](https://us.misumi-ec.com/vona2/detail/110300465870/)

### Option A: Cut yourself ($15)
| Qty | Length | Item |
|-----|--------|------|
| 2 | 2000mm | HFS3 15x15mm extrusion |

### Option B: Misumi pre-cut ($45)
| Qty | Length (mm) | Purpose |
|-----|-------------|---------|
| 4 | 385 | Main body box, lengthwise |
| 4 | 245 | Main body box, widthwise |
| 1 | 238 | Differential fixed beam |
| 2 | 182 | Rocker to front wheel |
| 2 | 161 | Rocker to bogie joint |
| 2 | 122 | Bogie to mid wheel |
| 2 | 117 | Bogie to rear wheel |

**Alternative:** MakerBeamXL (15x15mm) or AliExpress 1515 extrusion

**Subtotal: $15-45**

---

## 8. Shafts & Bearings (McMaster-Carr / Amazon)

| Qty | Item | Unit Price | Total | Source |
|-----|------|------------|-------|--------|
| 30 | 608 Bearings | $0.50 | $15 | [Amazon](https://amzn.to/3fMBiiG) |
| 1 | 8mm Aluminum Rod 500mm (x2) | $5 | $10 | McMaster #4634t34 |
| 1 | 8mm Steel Rod 300mm (differential) | $5 | $5 | McMaster #8920k26 |
| 100 | 5/16" E-Clips (retaining) | $8 | $8 | McMaster #97431a310 |
| 2 | Turnbuckles (M3 eyes) | $5 | $10 | [Amazon](https://amzn.to/3cCZ5Qg) |

**Subtotal: ~$48**

### Shaft Cut List
| Qty | Length | Purpose |
|-----|--------|---------|
| 6 | 50mm | Wheel axle drive shaft |
| 4 | 61mm | Steering shaft |
| 2 | 66.5mm | Bogie pivot |
| 2 | 84mm | Rocker pivot |
| 1 | 300mm | Differential shaft (steel) |

---

## 9. Hardware (Amazon / McMaster-Carr)

| Qty | Item | Est. Price | Source |
|-----|------|------------|--------|
| ~~300~~ | ~~M3x8mm socket head screws~~ | - | *OWNED* |
| ~~28~~ | ~~M3x16mm socket head screws~~ | - | *OWNED* |
| ~~300~~ | ~~M3 washers~~ | - | *OWNED* |
| ~~300~~ | ~~M3 nuts~~ | - | *OWNED* |
| 100 | M3 heat-set inserts | $15 | [Amazon](https://amzn.to/2WWCF5Q) |
| ~~22~~ | ~~M3x8mm set screws~~ | - | *OWNED* |
| 40 | 1/4" #2 Phillips brass screws | $5 | McMaster #98685a220 |
| 1 | Cable sleeve (Techflex) | $10 | [Amazon](https://amzn.to/366gdLy) |

**Subtotal: ~$30**

---

## 10. 3D Printing Filament

| Qty | Item | Unit Price | Total |
|-----|------|------------|-------|
| 4 | PLA/PETG 1kg spools | $25 | $100 |

**Note:** ~4kg needed for all parts including sensor mounts:
- Base Sawppy parts: ~2.5kg
- Sensor mounts (lidar tower, camera bracket, IMU enclosure): ~0.5kg
- Pi 5 tray and cable management: ~0.3kg
- Spares/reprints: ~0.7kg

**Subtotal: $100**

---

## 11. Optional Items

| Item | Price | Notes |
|------|-------|-------|
| Laser cut acrylic (equipment bay) | $10 | Or 3D print |
| Low battery alarm | $8 | [Amazon](https://amzn.to/3fVtOdt) |
| Wireless router (5GHz) | $30 | For crowded events |
| Arduino Nano (backup control) | $12 | [Amazon](https://amzn.to/3bvAFXy) |
| Joystick module | $6 | For wired backup |
| USB WiFi adapter (5GHz) | $15 | Better range for AI offload |

---

## Order by Vendor

### Amazon - Base Build (~$245)
- [ ] 2x LX-16A Bundle (10 servos + BusLinker) - $150
- [x] ~~Raspberry Pi 5~~ - *OWNED*
- [x] ~~microSD 32GB+~~ - *OWNED*
- [ ] 5V Regulator MP1584EN - $8
- [x] ~~JST connectors~~ - *OWNED*
- [x] ~~2S LiPo 5200mAh~~ - *OWNED*
- [x] ~~Deans T-Plug~~ - *OWNED*
- [ ] Fuse holder + fuses - $10
- [x] ~~LiPo charger~~ - *OWNED*
- [ ] Power switch (10A) - $6
- [ ] Voltage display - $6
- [ ] 608 bearings (30) - $15
- [ ] Turnbuckles - $10
- [ ] M3 heat-set inserts - $15
- [x] ~~M3 hardware assortment~~ - *OWNED*
- [ ] Cable sleeve - $10
- [ ] 4kg PLA/PETG filament - $100 *(or local)*

### Amazon - Autonomy Sensors (~$459)
- [ ] RPLidar A1 (12m 2D Lidar) - $99
- [ ] Intel RealSense D435i (Depth + IMU) - $300
- [ ] BNO055 IMU (or Adafruit) - $35
- [ ] USB 3.0 Powered Hub - $25

### Amazon - Autonomy Electronics & Wiring (~$59)
- [ ] DC-DC Buck 5V/5A USB-C (Pi power) - $12
- [ ] INA219 Battery Monitor (optional) - $8
- [ ] 14AWG Silicone Wire (red+black) - $10
- [ ] XT30 Connector Set (5 pairs) - $6
- [ ] JST 2.0 3P Extension Cables (10x 500mm) - $12
- [ ] USB-C PD Cable - $8
- [ ] I2C Logic Level Shifter - $3

### Misumi (~$15-45)
- [ ] HFS3 15x15mm extrusions (see cut list above)

### McMaster-Carr (~$28)
- [ ] 8mm aluminum rod 500mm x2 - $10
- [ ] 8mm steel rod 300mm - $5
- [ ] 5/16" E-clips (100) - $8
- [ ] 1/4" brass screws (40) - $5

---

## Printed Parts Checklist

### Base Sawppy Parts
| Qty | Part | Notes |
|-----|------|-------|
| 88 | Clip2n125 | Print extras |
| 12 | Clip3n20 | Print extras |
| 6 | Wheel Hub | Print extras |
| 10 | LX-16A Coupler | Print extras |
| 10 | LX-16A Bracket | |
| 6 | Wheel | |
| 4 | Steering Knuckle | |
| 4 | Body Corner | |
| 2 | DiffBrace | |
| 2 | DiffLink | |
| 1 | DiffLower | |
| 1 | DiffUpper | |
| 2 | Rod Support | |
| 1 | Power Panel | |
| 2 | Fixed Knuckle | Mirrored pair |
| 2 | Front Corner | Mirrored pair |
| 2 | Rear Corner | Mirrored pair |
| 2 | Bogie Wheels | Mirrored pair |
| 2 | Bogie Body | Mirrored pair |
| 2 | Rocker | Mirrored pair |
| 2 | Rocker Body Mount | Mirrored pair |
| 2 | DiffEnd | Mirrored pair |
| 2 | Battery Tray | Mirrored pair |

### Autonomy Sensor Mounts (Design/Print)
| Qty | Part | Notes |
|-----|------|-------|
| 1 | Lidar Tower | 150mm height, mounts to chassis top |
| 1 | Lidar Base Plate | Fits RPLidar A1 mounting holes |
| 1 | Camera Bracket | 180mm forward, 120mm above chassis |
| 1 | IMU Enclosure | Mounts to chassis top center |
| 1 | Pi 5 Mounting Tray | With ventilation slots |
| 1 | USB Hub Mount | |
| 10 | Cable Clips | For wire management |

---

## USB Port Allocation (Pi 5)

| Port | Device | Notes |
|------|--------|-------|
| USB 3.0 #1 | Intel RealSense D435i | Requires USB 3.0 bandwidth |
| USB 3.0 #2 | USB Hub (powered) | Expansion |
| USB 2.0 #1 | BusLinker (servos) | Serial @ 115200 baud |
| USB 2.0 #2 | RPLidar A1 | Built-in USB interface |
| I2C (GPIO) | BNO055 IMU | Via logic level shifter |

---

## Revision History

| Date | Change |
|------|--------|
| Dec 2025 | Initial shopping list |
| Dec 2025 | Verified Amazon links, updated to D435i |
| Jan 2026 | Added wiring section, removed unnecessary items (CH340, 5V/3A buck), increased filament to 4kg, added sensor mount prints, added USB port allocation |

---

*Generated: January 2026*
*Links verified: December 31, 2025*
*Source: Sawppy upstream docs + autonomy stack requirements*
