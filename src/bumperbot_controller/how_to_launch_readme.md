# Bumperbot Controller

This package provides the controller configuration and launch files for the Bumperbot robot in ROS 2.

## Prerequisites

- ROS 2 (Humble/Iron/Jazzy)
- `bumperbot_description` package
- Gazebo simulation environment
- `teleop_twist_keyboard` package (optional, for teleoperation)

## Build Instructions

1. Navigate to your workspace and build the packages:

   ```bash
   cd ~/bumperbot_ws
   colcon build
   ```

2. Source the workspace:

   ```bash
   source install/setup.bash
   ```

## Launch Instructions

### Step 1: Launch Gazebo Simulation

Open a terminal, source the workspace, and launch Gazebo with the Bumperbot world:

```bash
ros2 launch bumperbot_description gazebo.launch.py
```

### Step 2: Launch the Controller

In a new terminal (don't forget to source the workspace), launch the Bumperbot controller:

```bash
ros2 launch bumperbot_controller bumperbot_controller.launch.py
```

You should now see the Bumperbot in the Gazebo world with the controller running.

## Controlling the Robot


### Direct Topic Publishing

Send velocity commands directly to the controller:

```bash
ros2 topic pub /simple_velocity_controller/commands std_msgs/msg/Float64MultiArray "{layout: {dim: [], data_offset: 0}, data: [0.0, 1.0]}"
```

> **Note:** Adjust the `data` values `[left_wheel, right_wheel]` to control the robot's movement.

## Useful Commands

### List Active Controllers

```bash
ros2 control list_controllers
```

### Check Available Controller Interfaces

```bash
ros2 control list_hardware_interfaces
```

## Troubleshooting

- **Controller not spawning:** Ensure Gazebo is fully loaded before launching the controller.
- **Robot not moving:** Verify the controller is active using `ros2 control list_controllers`.
- **Source errors:** Make sure to source `install/setup.bash` in each new terminal.