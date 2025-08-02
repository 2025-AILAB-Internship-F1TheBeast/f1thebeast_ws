# Team F1TheBeast
> **2025 AILAB Summer Internship Program**  
> Jisang Yun / Subin Park / Jonghun Ann / Yongwoo Kwon / Yunsang Jeong

## Team Roll
> ### Perception
**Jisang Yun(Team Leader)**
- Project Managing
- Perception (SLAM & Localization)
- Planning (Support)
- Hardware

**Subin Park**
- Project Managing
- Perception (SLAM & Localization)

> ### Planning
**Jonghun Ahn**
- Valid Learning
- Minimum Curvature

**Yongwoo Kwon**
- Planning( Global & Learning)

> ### Control
**Yunsang Jeong**
<<<<<<< HEAD
- Control
- Done : Pure pursuit, Stanley, PID
- ToDo : MPC Study, Code
=======
- Hardware
- ToDo : Pure pursuit, Stanley, PID
>>>>>>> a6daf1735d92431111a22d29bfe265aa7c930581

## Basic Launch
```bash
ros2 launch f1tenth_stack bringup_launch.py
```
**Keyboard Teleop**
```bash
ros2 run key_teleop key_teleop  
```
- Update teleop switcher (with key 's' ackermann - twist mode)
- Speed Profile selection with numpad 1(0.6 m/s) - 9 - 0(4.0 m/s)

**SLAM-Toolbox**
```bash 
# slam-toolbox launch
ros2 launch slam_toolbox online_sync_launch.py slam_params_file:=./src/f1tenth_system/f1tenth_stack/config/slam_params.yaml
```

**Tips**
```
# F1Tenth aliases 
alias f110="ros2 launch f1tenth_stack bringup_launch.py"  # F1Tenth Stack Run 
alias key="ros2 run key_teleop key_teleop"  # Keyboard Control
```
Add this at ~/.bashrc or ~/.zshrc and source it for keyboard shortcut
