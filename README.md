# F1Tenth System Base Workspace
> **Keyboard Teleop included**

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
