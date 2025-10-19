# Bumperbot C++ Examples

This package contains C++ examples for ROS 2, demonstrating fundamental concepts for robotics development.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Package Structure](#package-structure)
- [Simple Publisher Tutorial](#simple-publisher-tutorial)
- [Building the Package](#building-the-package)
- [Running the Examples](#running-the-examples)
- [Understanding the Code](#understanding-the-code)

## Overview

The `bumperbot_cpp_examples` package provides practical examples of ROS 2 concepts implemented in C++. Currently, it includes:
- **Simple Publisher**: A basic publisher node that demonstrates message publishing in ROS 2

## Prerequisites

- ROS 2 (Humble, Iron, or later recommended)
- C++ compiler with C++14 support or later
- Basic understanding of ROS 2 concepts (nodes, topics, messages)

## Package Structure

```
bumperbot_cpp_examples/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── include/                # Header files (currently empty)
│   └── bumperbot_cpp_examples/
└── src/                    # Source files
    └── simple_publisher.cpp
```

## Simple Publisher Tutorial

### What is a Publisher?

A publisher is a ROS 2 node that sends messages to a specific topic. Other nodes can subscribe to this topic to receive these messages. This is the foundation of ROS 2's publish-subscribe communication pattern.

### Overview of simple_publisher

The `simple_publisher` node demonstrates:
- Creating a ROS 2 node using object-oriented programming
- Publishing messages to a topic
- Using timers for periodic callbacks
- Basic logging with `RCLCPP_INFO`

### Code Breakdown

#### 1. **Includes and Namespace**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;
```
- `rclcpp/rclcpp.hpp`: Core ROS 2 C++ client library
- `std_msgs/msg/string.hpp`: String message type
- `chrono`: For time-based operations
- `std::chrono_literals`: Enables time literals like `1s`

#### 2. **SimplePublisher Class**
```cpp
class SimplePublisher : public rclcpp::Node
```
The class inherits from `rclcpp::Node`, making it a ROS 2 node.

#### 3. **Constructor**
```cpp
SimplePublisher() : Node("simple_publisher"), counter_(0)
{
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
}
```
- **Node name**: "simple_publisher"
- **Publisher**: Creates a publisher on the "chatter" topic with a queue size of 10
- **Timer**: Sets up a 1-second timer that calls `timerCallback()` at 1 Hz
- **Logging**: Prints an info message when the node starts

#### 4. **Timer Callback**
```cpp
void timerCallback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello Ros2 Counter: " + std::to_string(counter_++);
    pub_->publish(message);
}
```
- Creates a new String message
- Sets the message data with an incrementing counter
- Publishes the message to the "chatter" topic

#### 5. **Main Function**
```cpp
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
- Initializes ROS 2
- Creates an instance of the SimplePublisher node
- Keeps the node running (spinning) to process callbacks
- Shuts down ROS 2 when interrupted

## Building the Package

### 1. Navigate to your workspace
```bash
cd ~/bumperbot_ws
```

### 2. Build the package
```bash
colcon build --packages-select bumperbot_cpp_examples
```

### 3. Source the workspace
```bash
source install/setup.bash
```

## Running the Examples

### Running the Simple Publisher

In a terminal, run:
```bash
ros2 run bumperbot_cpp_examples simple_publisher
```

**Expected Output:**
```
[INFO] [<timestamp>] [simple_publisher]: Publishing at 1 Hz
```

### Viewing Published Messages

In another terminal (don't forget to source your workspace), you can view the published messages using:

#### Option 1: Echo the topic
```bash
ros2 topic echo /chatter
```

**Expected Output:**
```
data: Hello Ros2 Counter: 0
---
data: Hello Ros2 Counter: 1
---
data: Hello Ros2 Counter: 2
---
...
```

#### Option 2: List active topics
```bash
ros2 topic list
```

You should see `/chatter` in the list.

#### Option 3: Get topic info
```bash
ros2 topic info /chatter
```

**Expected Output:**
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

#### Option 4: Check publishing rate
```bash
ros2 topic hz /chatter
```

**Expected Output:**
```
average rate: 1.000
```

## Understanding the Code

### Key Concepts

1. **Node Lifecycle**
   - Nodes are initialized with `rclcpp::init()`
   - They spin to process callbacks with `rclcpp::spin()`
   - They're shut down with `rclcpp::shutdown()`

2. **Publishers**
   - Created with `create_publisher<MessageType>(topic_name, queue_size)`
   - Queue size determines how many messages to buffer
   - Publish messages with `publisher->publish(message)`

3. **Timers**
   - Created with `create_wall_timer(duration, callback)`
   - Wall timers are real-time based (not simulation time)
   - Callbacks are executed periodically

4. **Smart Pointers**
   - ROS 2 C++ uses `std::shared_ptr` for memory management
   - Prevents memory leaks and makes code safer

### Common Modifications

#### Change Publishing Rate
Modify the timer duration in the constructor:
```cpp
// Publish at 2 Hz (every 500ms)
timer_ = create_wall_timer(500ms, std::bind(&SimplePublisher::timerCallback, this));
```

#### Change Topic Name
Modify the publisher creation:
```cpp
pub_ = create_publisher<std_msgs::msg::String>("my_topic", 10);
```

#### Change Message Content
Modify the `timerCallback()` function:
```cpp
message.data = "Custom message: " + std::to_string(counter_++);
```

#### Change Queue Size
A larger queue size buffers more messages:
```cpp
pub_ = create_publisher<std_msgs::msg::String>("chatter", 100);
```

## Troubleshooting

### Build Errors
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Clean build: `rm -rf build install log` then rebuild

### Runtime Errors
- **Node not found**: Make sure you've sourced the workspace (`source install/setup.bash`)
- **Topic not appearing**: Check if the node is running (`ros2 node list`)

## Next Steps

After understanding the simple publisher, you can:
1. Create a subscriber node to receive these messages
2. Implement parameter support for configuring the node
3. Add custom message types
4. Implement services and actions
5. Create launch files for easier node management

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [rclcpp API Documentation](https://docs.ros2.org/latest/api/rclcpp/)

## License

TODO: Add license information

## Maintainer

parallels@todo.todo
