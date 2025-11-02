# Visualizing Robot with RViz2

This guide explains how to visualize the Bumperbot robot model in RViz2.

## Prerequisites

Ensure you have built the workspace:
```bash
cd /home/parallels/bumperbot_ws
colcon build
```

## Steps to Visualize

1. **Source the workspace setup file:**
   ```bash
   source install/setup.bash
   ```

2. **Start the robot state publisher:**
   ```bash
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/parallels/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"
   ```

3. **Launch the joint state publisher GUI (in a new terminal):**
   ```bash
   source install/setup.bash
   ros2 run joint_state_publisher_gui joint_state_publisher_gui
   ```

4. **Start RViz2 (in another new terminal):**
   ```bash
   source install/setup.bash
   ros2 run rviz2 rviz2
   ```

## Configuring RViz2

Once RViz2 is running:

1. Click **Add** button in the Displays panel
2. Add **TF** display to visualize coordinate frames
3. Add **RobotModel** display to visualize the robot
4. In the RobotModel display settings, ensure the **Description Topic** is set to `/robot_description`
5. Set **Fixed Frame** to an appropriate frame (e.g., `base_link` or `base_footprint`)

The robot model should now be visible in RViz2.