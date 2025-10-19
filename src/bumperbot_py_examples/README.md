# Bumperbot Python Examples Tutorial

Welcome to the **bumperbot_py_examples** package! This tutorial will guide you through creating and running ROS 2 Python nodes, starting with a simple publisher example.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Package Structure](#package-structure)
- [Understanding the Simple Publisher](#understanding-the-simple-publisher)
- [Building the Package](#building-the-package)
- [Running the Node](#running-the-node)
- [What's Happening?](#whats-happening)
- [Monitoring the Output](#monitoring-the-output)
- [Next Steps](#next-steps)

## Overview

This package contains Python-based ROS 2 examples designed to help you learn ROS 2 fundamentals. Currently, it includes:

- **simple_publisher**: A basic ROS 2 publisher node that demonstrates how to publish messages to a topic at a regular interval.

## Prerequisites

Before you begin, make sure you have:

- ROS 2 installed (Humble, Iron, or Jazzy recommended)
- Basic Python knowledge
- A ROS 2 workspace set up (this package is part of `bumperbot_ws`)
- `colcon` build tool installed

## Package Structure

```
bumperbot_py_examples/
├── bumperbot_py_examples/          # Python package directory
│   ├── __init__.py                 # Package initialization
│   └── simple_publisher.py         # Simple publisher node
├── resource/                       # Resource marker files
│   └── bumperbot_py_examples
├── test/                           # Unit tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                     # Package metadata and dependencies
├── setup.py                        # Python package setup configuration
└── setup.cfg                       # Setup configuration
```

## Understanding the Simple Publisher

The `simple_publisher.py` file contains a basic ROS 2 publisher node. Let's break down what it does:

### Code Breakdown

```python
class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")
```

- Creates a ROS 2 node named `"simple_publisher"`
- Inherits from the `Node` class provided by `rclpy`

```python
self.pub_ = self.create_publisher(String, "chatter", 10)
```

- Creates a publisher that publishes `String` messages
- Topic name: `"chatter"`
- Queue size: `10` (stores up to 10 messages if subscribers can't keep up)

```python
self.counter_ = 0
self.frequency_ = 1.0  # 1 Hz
```

- Initializes a counter to track how many messages have been sent
- Sets the publishing frequency to 1 Hz (once per second)

```python
self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
```

- Creates a timer that calls `timerCallback()` every 1 second

```python
def timerCallback(self):
    msg = String()
    msg.data = "Hello Ros2! - counter: %d " % self.counter_
    self.pub_.publish(msg)
    self.counter_ += 1
```

- Creates a new `String` message
- Sets the message data with a counter value
- Publishes the message to the `"chatter"` topic
- Increments the counter

### Key Concepts

1. **Publisher**: A ROS 2 entity that sends messages to a topic
2. **Topic**: A named channel for message communication
3. **Message Type**: `std_msgs/String` - a standard message containing text data
4. **Timer**: Executes a callback function at regular intervals

## Building the Package

Navigate to your workspace root and build the package:

```bash
cd ~/bumperbot_ws
colcon build --packages-select bumperbot_py_examples
```

After building, source the workspace:

```bash
source install/setup.bash
```

## Running the Node

To run the simple publisher node:

```bash
ros2 run bumperbot_py_examples simple_publisher
```

You should see output like:

```
[INFO] [<timestamp>] [simple_publisher]: Publishing at 1 Hz
```

The node is now running and publishing messages to the `chatter` topic every second!

## What's Happening?

When you run the `simple_publisher` node:

1. **Initialization**: The node starts and creates a publisher on the `chatter` topic
2. **Timer Setup**: A timer is configured to trigger every 1 second
3. **Publishing Loop**: Every second, the timer callback:
   - Creates a message: `"Hello Ros2! - counter: X"`
   - Publishes it to the `chatter` topic
   - Increments the counter

## Monitoring the Output

You can verify that messages are being published using several methods:

### Method 1: Echo the Topic

In a new terminal (remember to source the workspace first):

```bash
source ~/bumperbot_ws/install/setup.bash
ros2 topic echo /chatter
```

You'll see the messages being published:

```
data: 'Hello Ros2! - counter: 0 '
---
data: 'Hello Ros2! - counter: 1 '
---
data: 'Hello Ros2! - counter: 2 '
---
```

### Method 2: List Active Topics

```bash
ros2 topic list
```

You should see `/chatter` in the list of active topics.

### Method 3: Check Topic Information

```bash
ros2 topic info /chatter
```

This shows details about the topic, including:
- Message type: `std_msgs/msg/String`
- Number of publishers: 1
- Number of subscribers: 0 (or more if you're running `ros2 topic echo`)

### Method 4: Monitor Publishing Rate

```bash
ros2 topic hz /chatter
```

This will show the actual publishing frequency (should be around 1 Hz).

## Next Steps

Now that you understand the basics of a ROS 2 publisher, try these exercises:

### Exercise 1: Modify the Publishing Rate

Change the `frequency_` value to publish faster or slower:

```python
self.frequency_ = 2.0  # 2 Hz (twice per second)
```

Rebuild and run to see the difference!

### Exercise 2: Customize the Message

Modify the message content in `timerCallback()`:

```python
msg.data = "Custom message #%d from my robot!" % self.counter_
```

### Exercise 3: Add More Information

Use `get_logger().info()` to print when each message is published:

```python
def timerCallback(self):
    msg = String()
    msg.data = "Hello Ros2! - counter: %d " % self.counter_
    self.pub_.publish(msg)
    self.get_logger().info("Published: '%s'" % msg.data)
    self.counter_ += 1
```

### Exercise 4: Create a Subscriber

As a next challenge, create a `simple_subscriber.py` node that listens to the `chatter` topic and prints received messages. This will help you understand the publisher-subscriber pattern in ROS 2.

### Exercise 5: Publish Different Message Types

Explore other message types from `std_msgs`:
- `Int32` for integer values
- `Float64` for decimal numbers
- `Bool` for true/false values

Check available message types:

```bash
ros2 interface list | grep std_msgs
```

## Troubleshooting

### Issue: Node not found

**Error**: `Package 'bumperbot_py_examples' not found`

**Solution**: Make sure you've built the package and sourced the workspace:

```bash
cd ~/bumperbot_ws
colcon build --packages-select bumperbot_py_examples
source install/setup.bash
```

### Issue: Import errors

**Error**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**: Ensure ROS 2 is properly installed and sourced:

```bash
source /opt/ros/<your-ros-distro>/setup.bash
```

### Issue: Permission denied

**Error**: Permission errors when running the node

**Solution**: Ensure the Python file is executable (this should be handled by the build system, but you can verify):

```bash
chmod +x ~/bumperbot_ws/src/bumperbot_py_examples/bumperbot_py_examples/simple_publisher.py
```

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [rclpy API Documentation](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [std_msgs Documentation](https://docs.ros2.org/latest/api/std_msgs/)

## Contributing

Feel free to add more example nodes to this package! Some ideas:
- Subscriber nodes
- Service servers and clients
- Action servers and clients
- Parameter usage examples
- Launch file examples

---

Happy coding with ROS 2! 🤖
