# Bumperbot Joystick Teleoperation

This guide explains how to control the Bumperbot robot using a joystick/gamepad for teleoperation in ROS 2.

## Prerequisites

- ROS 2 (Humble/Iron/Jazzy)
- `bumperbot_description` package
- `bumperbot_controller` package
- Gazebo simulation environment
- Joystick/gamepad (Xbox, PlayStation, or compatible controller)
- Required ROS 2 packages:
  ```bash
  sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joy-teleop
  ```

## Quick Start

1. **Build the workspace:**
   ```bash
   cd ~/bumperbot_ws
   colcon build
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

3. **Launch Gazebo simulation:**
   ```bash
   ros2 launch bumperbot_description gazebo.launch.py
   ```

4. **Launch the robot controller:**
   ```bash
   ros2 launch bumperbot_controller controller.launch.py
   ```

5. **Launch joystick teleoperation:**
   ```bash
   ros2 launch bumperbot_controller joystick_teleop.launch.py
   ```

## Joystick Controls

### Movement Controls
- **Left Stick (Up/Down)**: Linear velocity (forward/backward)
  - Push up: Move forward
  - Push down: Move backward
- **Right Stick (Left/Right)**: Angular velocity (rotation)
  - Push left: Turn left
  - Push right: Turn right

### Safety Feature
- **Deadman Button**: Hold **RB/R1 button (Button 5)** while driving
- **Release the deadman button** and the robot will stop immediately

### Control Parameters
- **Linear speed scale**: 0.5 (adjustable in `joy_teleop.yaml`)
- **Angular speed scale**: 2.0 (adjustable in `joy_teleop.yaml`)

## Configuration Files

### Joystick Configuration
- **File**: `bumperbot_controller/config/joy_config.yaml`
- **Purpose**: Configures joystick device settings and button mappings

### Teleoperation Configuration
- **File**: `bumperbot_controller/config/joy_teleop.yaml`
- **Purpose**: Maps joystick inputs to robot velocity commands
- **Topic**: Publishes to `bumperbot_controller/cmd_vel`

## Troubleshooting

### Joystick Not Detected
```bash
# Check if joystick is connected
ls /dev/input/js*
# Or check with jstest
sudo apt install joystick
jstest /dev/input/js0
```

### No Movement
1. **Check deadman button**: Ensure you're holding the RB/R1 button
2. **Verify controller status**:
   ```bash
   ros2 control list_controllers
   ```
3. **Check joystick topics**:
   ```bash
   ros2 topic list | grep joy
   ros2 topic echo /joy
   ```

### Permission Issues
```bash
# Add user to input group
sudo usermod -a -G input $USER
# Logout and login again, or reboot
```

### Customizing Controls
Edit `bumperbot_controller/config/joy_teleop.yaml` to adjust:
- Speed scaling
- Axis mappings
- Button assignments

## Alternative Control Methods

If joystick is unavailable, you can also control the robot using:
- **Keyboard**: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- **Direct topic publishing**: `ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'`

## Tips

- Start with gentle joystick movements to avoid jerky robot motion
- The deadman button provides safety by requiring constant pressure to move
- Monitor the robot in Gazebo to ensure proper operation
- Adjust speed scales in the config file for your preferred control feel
