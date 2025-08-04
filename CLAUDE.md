# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 F1TENTH autonomous racing workspace for Team F1TheBeast (2025 AILAB Summer Internship Program). The project includes perception (SLAM & localization), planning, and control systems for autonomous racing.

## Common Development Commands

### Building the Workspace
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Source the workspace after building
source install/setup.bash
```

### Testing
```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>
```

### Main Launch Commands
```bash
# Launch the main F1TENTH stack
ros2 launch f1tenth_stack bringup_launch.py

# Keyboard teleop control
ros2 run key_teleop key_teleop

# SLAM mapping
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=./src/f1tenth_system/f1tenth_stack/config/slam_params.yaml

# Particle filter localization
ros2 launch particle_filter localize_launch.py

# Simulation bridge (for F1TENTH gym)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## Project Architecture

### Core Components

1. **f1tenth_stack** - Main system integration package containing:
   - `bringup_launch.py` - Primary launch file that starts all essential nodes
   - Configuration files for sensors, VESC motor controller, joystick, and ackermann multiplexer
   - Python nodes for TF publishing and throttle interpolation

2. **ackermann_mux** - Multiplexes ackermann drive commands from different sources (teleop vs autonomous navigation) with priority-based switching

3. **VESC System** - Motor control stack:
   - `vesc_driver` - Low-level VESC communication
   - `vesc_ackermann` - Converts between Ackermann commands and VESC motor/servo commands

4. **Teleop Tools** - Multiple teleoperation options:
   - `joy_teleop` - Joystick control (Logitech F-710 with deadman switches)
   - `key_teleop` - Keyboard control with speed profiles and mode switching
   - `mouse_teleop` - Mouse-based control

5. **Perception System**:
   - `particle_filter` - Monte Carlo Localization using RangeLibc for fast ray casting
   - SLAM-toolbox integration for mapping
   - Hokuyo LiDAR integration via `urg_node`

6. **f1tenth_gym_ros** - Simulation bridge connecting F1TENTH gym environment to ROS 2

### Configuration Files

Key configuration files in `src/f1tenth_system/f1tenth_stack/config/`:
- `slam_params.yaml` - SLAM-toolbox parameters for mapping
- `mux.yaml` - Ackermann command multiplexer settings (navigation priority: 10, joystick: 100)
- `vesc.yaml` - Motor controller parameters
- `sensors.yaml` - Sensor configuration
- `joy_teleop.yaml` - Joystick mapping and parameters

### Key Topics

**Subscribed by the system:**
- `/drive` - Autonomous navigation commands (AckermannDriveStamped)
- `/teleop` - Teleop commands from joystick/keyboard

**Published by the system:**
- `/scan` - LiDAR data (LaserScan)
- `/odom` - Odometry data
- `/sensors/imu/raw` - IMU data
- `/sensors/core` - VESC telemetry

### Hardware Setup

The system is designed for F1TENTH race cars with:
- VESC motor controller
- Hokuyo LiDAR
- IMU sensor
- Logitech F-710 gamepad (LB: teleop deadman, RB: navigation deadman)

### Development Notes

- This is a colcon-based ROS 2 workspace
- The current branch is `perception`; main development branch is `master`
- Built packages are in `install/` directory
- Log files are stored in `log/` directory
- SLAM maps are saved in `slam-maps/` directory
- ROS bag recordings are in `rosbag_folder/`

### Useful Aliases
Add to ~/.bashrc or ~/.zshrc:
```bash
alias f110="ros2 launch f1tenth_stack bringup_launch.py"
alias key="ros2 run key_teleop key_teleop"
```

### Team Roles
- **Perception Team**: SLAM, localization, and sensor integration
- **Planning Team**: Path planning algorithms (global & learning-based)
- **Control Team**: Low-level vehicle control (Pure Pursuit, Stanley, PID)