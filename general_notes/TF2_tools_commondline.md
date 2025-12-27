# TF2 Tools Command Line Usage

This document provides a comprehensive guide to using TF2 (Transform Library) command-line tools in ROS 2. TF2 is a library for keeping track of coordinate frames and transforms between them over time. The tools demonstrated here help visualize, inspect, and debug transform trees in robotic systems.

## Prerequisites

Before running these commands, ensure you have:
- ROS 2 installed and sourced (`source install/setup.bash`)
- A running ROS 2 system with TF2 publishers broadcasting transforms
- The `tf2_tools` and `tf2_ros` packages installed

## Tools Overview

### 1. `view_frames` (from tf2_tools)
Visualizes the current transform tree structure and generates graphical representations.

### 2. `tf2_echo` (from tf2_ros)
Prints the transform between two coordinate frames in real-time.

---

## 1. view_frames Tool

### Description
The `view_frames` tool listens to TF2 data for a short period (default 5 seconds) and generates a visual representation of the transform tree. It creates both a PDF diagram and a GraphViz (.gv) file showing the relationships between coordinate frames.

### Command
```bash
ros2 run tf2_tools view_frames
```

### Output Explanation

#### Listening Phase
```
[INFO] [1766865013.487922785] [view_frames]: Listening to tf data for 5.0 seconds...
```
- The tool collects TF2 transform data for 5 seconds to build a complete picture of the transform tree.

#### Graph Generation
```
[INFO] [1766865018.552462128] [view_frames]: Generating graph...
```
- After collecting data, it analyzes the relationships between frames to create a graph structure.

#### Frame Information
```
[INFO] [1766865018.557879915] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="bumperbot_base: \n  parent: 'odom'\n  broadcaster: 'default_authority'\n  rate: 10.199\n  most_recent_transform: 1766865018.550683\n  oldest_transform: 1766865013.550177\n  buffer_length: 5.001\nbumperbot_top: \n  parent: 'bumperbot_base'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n")
```

**Frame Details:**
- **bumperbot_base**:
  - `parent: 'odom'` - This frame is a child of the 'odom' frame
  - `broadcaster: 'default_authority'` - The node publishing this transform
  - `rate: 10.199` - Transform publication rate in Hz
  - `most_recent_transform: 1766865018.550683` - Timestamp of latest transform (Unix time)
  - `oldest_transform: 1766865013.550177` - Timestamp of oldest buffered transform
  - `buffer_length: 5.001` - Time span of buffered transforms in seconds

- **bumperbot_top**:
  - `parent: 'bumperbot_base'` - Child of bumperbot_base frame
  - `rate: 10000.000` - Very high rate (possibly static transform)
  - `buffer_length: 0.000` - No buffering, likely a static transform

#### File Export
```
[INFO] [1766865018.560835492] [view_frames]: Exporting graph in frames_2025-12-27_22.50.18.pdf file...
```
- Creates a PDF visualization of the transform tree
- Also generates a GraphViz (.gv) file for further processing

#### Generated Files
```
parallels@ubuntu-linux-22-04-desktop:~/bumperbot_ws$ ls
build                          frames_2025-12-27_22.50.18.pdf  install  src
frames_2025-12-27_22.50.18.gv  general_notes                   log
```
- `frames_2025-12-27_22.50.18.pdf` - Visual diagram of the transform tree
- `frames_2025-12-27_22.50.18.gv` - GraphViz source file

---

## 2. tf2_echo Tool

### Description
The `tf2_echo` tool continuously prints the transform between two specified coordinate frames. It shows translation, rotation (in multiple formats), and the full transformation matrix. This is useful for debugging transform relationships and verifying coordinate frame alignments.

### Command Format
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
```

---

## 2.1 tf2_echo: bumperbot_base to bumperbot_top

### Command
```bash
ros2 run tf2_ros tf2_echo bumperbot_base bumperbot_top
```

### Output Explanation

#### Initial Warning
```
[INFO] [1766865207.742159078] [tf2_echo]: Waiting for transform bumperbot_base ->  bumperbot_top: Invalid frame ID "bumperbot_base" passed to canTransform argument target_frame - frame does not exist
```
- The tool initially reports that the source frame doesn't exist
- This can happen if transforms haven't been published yet or frames are not active

#### Transform Data
```
At time 0.0
- Translation: [0.000, 0.000, 0.300]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.000
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.300
  0.000  0.000  0.000  1.000
```

**Transform Components:**
- **Time**: `0.0` - Transform timestamp (0.0 often indicates a static transform)
- **Translation**: `[0.000, 0.000, 0.300]` - X, Y, Z displacement in meters
  - No movement in X or Y directions
  - 0.3 meters offset in Z direction (upward)
- **Rotation (Quaternion)**: `[0.000, 0.000, 0.000, 1.000]` - Unit quaternion (xyzw format)
  - Represents no rotation (identity quaternion)
- **Rotation (RPY - Radians)**: `[0.000, -0.000, 0.000]` - Roll, Pitch, Yaw in radians
  - No rotation in any axis
- **Rotation (RPY - Degrees)**: `[0.000, -0.000, 0.000]` - Same values in degrees
- **Transformation Matrix**: 4x4 homogeneous transformation matrix
  - Top-left 3x3: Rotation matrix (identity = no rotation)
  - Top-right 3x1: Translation vector
  - Bottom row: [0, 0, 0, 1] (homogeneous coordinates)

This transform represents a static offset of 30cm upward with no rotation.

---

## 2.2 tf2_echo: odom to bumperbot_top

### Command
```bash
ros2 run tf2_ros tf2_echo odom bumperbot_top
```

### Output Explanation

#### Initial Warning
```
[INFO] [1766865358.725326034] [tf2_echo]: Waiting for transform odom ->  bumperbot_top: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
```
- Similar initial warning about the source frame

#### Dynamic Transform Data
The output shows multiple transforms over time, indicating a moving robot:

```
At time 1766865359.651946972
- Translation: [195.800, 0.000, 0.300]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, -0.863, 0.505]
- Rotation: in RPY (radian) [0.000, 0.000, -2.083]
- Rotation: in RPY (degree) [0.000, 0.000, -119.358]
- Matrix:
 -0.490  0.872  0.000 195.800
 -0.872 -0.490 -0.000  0.000
 -0.000  0.000  1.000  0.300
  0.000  0.000  0.000  1.000
```

**Analysis of Movement:**
- **X Translation**: Increases from 195.8m to 197.8m over time (robot moving forward)
- **Y Translation**: Remains 0.0m (straight line motion)
- **Z Translation**: Constant 0.3m (height offset)
- **Rotation**: Changes significantly, indicating the robot is rotating
  - Yaw angle changes from -119.358° to 126.051°
  - The robot is spinning while moving forward

#### Key Observations
1. **Linear Motion**: Robot moves along X-axis at approximately 0.5m/s
2. **Rotational Motion**: Continuous rotation, completing multiple full turns
3. **Consistent Height**: Z-offset remains constant at 0.3m
4. **2D Motion**: No Y-axis movement (planar motion)

#### Transformation Matrix Breakdown
The 4x4 matrix represents the full rigid body transformation:
- **Rotation part** (top-left 3x3): Rotates coordinates from odom to bumperbot_top frame
- **Translation part** (top-right 3x1): Translates coordinates after rotation
- **Homogeneous row** (bottom): Standard [0,0,0,1] for matrix multiplication

---

## Usage Tips

1. **view_frames**: Run when setting up transforms to verify the tree structure
2. **tf2_echo**: Use for debugging specific transform relationships
3. **Static vs Dynamic**: Time 0.0 indicates static transforms, non-zero times indicate dynamic transforms
4. **Frame Existence**: Warnings about non-existent frames often resolve as transforms are published
5. **Visualization**: Open the generated PDF to see the complete transform tree graphically

## Common Issues

- **Frame not found**: Ensure TF2 publishers are running and frames are being broadcast
- **Empty buffer**: Static transforms may show zero buffer length
- **High rates**: Very high publication rates (>1000Hz) often indicate static transforms
