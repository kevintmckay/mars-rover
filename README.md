# Mars Rover (Sawppy)

Autonomous Mars rover based on [Sawppy](https://github.com/Roger-random/Sawppy_Rover) with ROS2 Jazzy, SLAM, Nav2 navigation, and AI reasoning offloaded to a local GPU server.

## Overview

A 6-wheel rocker-bogie rover inspired by NASA's Mars rovers, upgraded with full autonomous capabilities:

- **ROS2 Jazzy** on Raspberry Pi 5
- **LX-16A serial bus servos** (10 total: 6 drive + 4 steering)
- **Ackermann steering geometry** with 4-corner steering
- **SLAM & Nav2** for autonomous navigation
- **AI offloading** to MacBook M3 Max via encrypted WiFi

## Specifications

| Spec | Value |
|------|-------|
| Wheels | 6 (rocker-bogie suspension) |
| Steering | 4-corner Ackermann |
| Drive | LX-16A servos in motor mode |
| Max Speed | 0.5 m/s |
| Wheelbase | 0.28m (front to rear) |
| Track | 0.20m (front/rear), 0.27m (mid) |
| Wheel Radius | 0.06m |
| Brain | Raspberry Pi 5 (8GB/16GB) |
| OS | Ubuntu 24.04 + ROS2 Jazzy |

## Hardware

### Sensors (for autonomy)
- RPLidar A1 (2D, 12m range)
- Intel RealSense D435 (depth camera)
- BNO055 IMU (9-axis)

### Servos
| Function | Servo IDs |
|----------|-----------|
| Drive FL, FR, ML, MR, RL, RR | 25, 27, 21, 22, 20, 28 |
| Steer FL, FR, RL, RR | 23, 29, 24, 26 |

## ROS2 Packages

```
ros2_ws/src/
├── sawppy_description/    # URDF model
├── sawppy_hardware/       # LX-16A servo driver
├── sawppy_control/        # Ackermann kinematics
├── sawppy_sensors/        # Sensor configs
├── sawppy_localization/   # EKF sensor fusion
├── sawppy_navigation/     # Nav2 & SLAM
├── sawppy_ai/             # AI offloading to Ollama
└── sawppy_bringup/        # Launch files
```

## Quick Start

### Build
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### Test Servos
```bash
# Scan for connected servos
ros2 run sawppy_hardware lx16a_test --port /dev/ttyUSB0 scan

# Check Sawppy servo status
ros2 run sawppy_hardware lx16a_test --port /dev/ttyUSB0 status
```

### Teleop
```bash
ros2 launch sawppy_bringup teleop.launch.py
```

### Full Autonomy
```bash
ros2 launch sawppy_bringup sawppy.launch.py
```

### View URDF
```bash
ros2 launch sawppy_description display.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | Twist | Velocity commands |
| `/joint_states` | JointState | Wheel positions/velocities |
| `/odom` | Odometry | Robot odometry |
| `/scan` | LaserScan | Lidar data |
| `/sawppy/ai/command` | String | Natural language commands |
| `/sawppy/ai/response` | String | AI responses (JSON) |

## AI Offloading

Complex reasoning is offloaded to a MacBook M3 Max running Ollama:

```bash
# On MacBook
OLLAMA_HOST=0.0.0.0 ollama serve

# Models
ollama pull qwen2.5:32b-instruct-q4_K_M  # Complex reasoning
ollama pull qwen2.5:14b-instruct          # Fast responses
```

### Supported AI Tasks
- Scene analysis
- Terrain traversability assessment
- Natural language commands
- Path planning assistance
- Decision making

## Shopping List

See [SHOPPING_LIST.md](SHOPPING_LIST.md) for complete parts list.

| Category | Cost |
|----------|------|
| Base Sawppy Build | ~$358-388 |
| Autonomy Sensors | ~$493 |
| **Total** | **~$851-881** |

## Project Structure

```
.
├── README.md
├── SHOPPING_LIST.md
├── ros2_ws/              # ROS2 workspace
│   └── src/              # Source packages
└── upstream/             # Original Sawppy repo (reference)
```

## References

- [Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover) - Original design by Roger Cheng
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/) - Robot Operating System
- [Nav2](https://docs.nav2.org/) - Navigation stack
- [Ollama](https://ollama.ai/) - Local LLM inference

## License

MIT License - Based on Sawppy by Roger Cheng
