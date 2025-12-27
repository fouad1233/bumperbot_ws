# BumperBot Workspace

This repository contains the ROS 2 workspace for the BumperBot project a mobile robot platform for learning and experimenting with ROS 2, simulation, and robot control.

## Project Overview

BumperBot is a differential-drive robot designed for simulation and real-world experiments. The workspace provides:

- Robot description (URDF, meshes, simulation configs)
- Controllers and teleoperation
- Example nodes in both Python and C++
- Custom ROS 2 messages and services
- Documentation and launch instructions

---

## Workspace Structure and Package Details

### 1. `bumperbot_description`

- **Purpose:** Defines the robot's physical model, simulation properties, and visualization tools.
- **Key files:**
  - `urdf/bumperbot.urdf.xacro`: Main robot description (links, joints, sensors, colors).
  - `urdf/bumperbot_ros2_control.xacro`: Hardware interface for simulation/real robot.
  - `urdf/bumperbot.gazebo.xacro`: Gazebo-specific simulation properties.
  - `meshes/`: 3D models for robot parts.
  - `launch/gazebo.launch.py`, `launch/display.launch.py`: Launch files for simulation and visualization.
  - `urdf/visualize_with_rviz.md`, `urdf/how_to_simple_visualise.md`: Step-by-step guides for visualizing the robot in RViz2.

### 2. `bumperbot_controller`

- **Purpose:** Provides controller nodes, configuration, and launch files for operating the robot.
- **Key files:**
  - `bumperbot_controller/simple_controller.py`: Example Python controller node for converting velocity commands to wheel commands.
  - `config/bumperbot_controllers.yaml`: Controller manager and diff drive controller configuration.
  - `launch/controller.launch.py`, `launch/joystick_teleop.launch.py`: Launch files for starting controllers and joystick teleoperation.
  - `how_to_launch_readme.md`, `launch/joystick_teleop_readme.md`: Detailed usage and teleoperation instructions.

### 3. `bumperbot_msgs`

- **Purpose:** Defines custom ROS 2 messages and services for the BumperBot ecosystem.
- **Key files:**
  - `srv/AddTwoInts.srv`: Example service for adding two integers.
  - `srv/GetTransform.srv`: Service for requesting a transform between frames.

### 4. `bumperbot_cpp_examples`

- **Purpose:** C++ example nodes demonstrating ROS 2 concepts.
- **Key files:**
  - `src/simple_publisher.cpp`: Publishes string messages to a topic.
  - `src/simple_subscriber.cpp`, `src/simple_service_server.cpp`, etc.: Additional C++ ROS 2 examples.
  - `README.md`: In-depth tutorials and explanations for each example.

### 5. `bumperbot_py_examples`

- **Purpose:** Python example nodes for learning ROS 2 basics.
- **Key files:**
  - `bumperbot_py_examples/simple_publisher.py`: Publishes string messages to a topic.
  - `bumperbot_py_examples/simple_service_server.py`, etc.: Additional Python ROS 2 examples.
  - `README.md`: Step-by-step tutorials for each example.

### 6. `general_notes`

- **Purpose:** Contains documentation on ROS 2 concepts, TF2, and other robotics topics.
- **Key files:**
  - `Angle_representations.md`, `TF2_operations_with_transformations.md`, etc.: Notes and guides.
  - `image/`: Images for documentation.

---

## How to Use

1. **Clone the repository:**
   ```bash
   git clone <this-repo-url>
   cd bumperbot_ws
   ```
2. **Build the workspace:**
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   colcon build
   ```
3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```
4. **Run simulation, controllers, or example nodes** as described in each package's README or launch file.

---

## Example: Simulate and Control BumperBot

1. **Launch simulation:**
   ```bash
   ros2 launch bumperbot_description gazebo.launch.py
   ```
2. **Start the controller:**
   ```bash
   ros2 launch bumperbot_controller controller.launch.py
   ```
3. **Teleoperate with a joystick:**
   ```bash
   ros2 launch bumperbot_controller joystick_teleop.launch.py
   ```

---

## Notes

- The `src/` directory contains all source packages.
- The `build/`, `install/`, and `log/` directories are auto-generated.
- See each package's README for more details and tutorials.
- The workspace is compatible with ROS 2 Humble, Iron, and Jazzy.

---
