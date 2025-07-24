# Team F1TheBeast
> **2025 AILAB Summer Internship Program**  
> Jisang Yun / Subin Park / Jonghun Ann / Yongwoo Kwon / Yunsang Jeong

## Team Roll
### Perception
**Jisang Yun(Team Leader)**
- Project Managing
- Perception (SLAM & Localization)
- Planning (Support)
- Hardware

**Subin Park**
- Project Managing
- Perception (SLAM & Localization)

### Planning
**Jonghun Ahn**
- Valid Learning
- Minimum Curvature

**Yongwoo Kwon**
- Add here
- 

### Control
**Yunsang Jeong**
- Control
- ToDo : Pure pursuit, Stanley, PID

## Basic Launch
```bash
ros2 launch f1tenth_stack bringup_launch.py
```
**Keyboard Teleop**
```bash
ros2 run key_teleop key_teleop  
```
**Tips**
```
# F1Tenth aliases 
alias f110="ros2 launch f1tenth_stack bringup_launch.py"  # F1Tenth Stack Run 
alias key="ros2 run key_teleop key_teleop"  # Keyboard Control
```
Add this at ~/.bashrc or ~/.zshrc and source it for keyboard shortcut
