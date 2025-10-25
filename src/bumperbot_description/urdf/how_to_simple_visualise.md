# How to Visualize the Bumperbot Model

This guide explains how to visualize the bumperbot URDF model in RViz2.

## Prerequisites

Make sure you have the `urdf_tutorial` package installed:
```bash
sudo apt install ros-jazzy-urdf-tutorial
```

## Visualization Steps

1. **Navigate to the workspace directory:**
   ```bash
   cd ~/bumperbot_ws/
   ```

2. **Source the workspace:**
   ```bash
   source ./install/setup.bash
   ```

3. **Launch the URDF visualization:**
   ```bash
   ros2 launch urdf_tutorial display.launch.py model:=/home/parallels/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro
   ```

   This command will:
   - Parse the URDF/xacro file
   - Start the `robot_state_publisher` node
   - Launch RViz2 with the robot model visualization
   - Provide a joint state publisher GUI to control robot joints

## Troubleshooting

### RViz Shows a Black Screen

If RViz displays a black screen or rendering issues, this is typically related to graphics drivers. Add the following line to your `~/.bashrc` file:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Then reload your terminal:
```bash
source ~/.bashrc
```

**Note:** This forces software rendering instead of hardware acceleration. While it solves display issues, it may result in slower rendering performance.

### Alternative: Using Relative Path

You can also use a relative path instead of an absolute path:
```bash
ros2 launch urdf_tutorial display.launch.py model:=src/bumperbot_description/urdf/bumperbot.urdf.xacro
```

This works as long as you're in the workspace root directory (`~/bumperbot_ws/`).