# ROS2 Parameter Management Guide

This guide provides comprehensive coverage of managing ROS2 parameters via terminal commands.

## Table of Contents
- [Building and Running](#building-and-running)
- [Listing Parameters](#listing-parameters)
- [Getting Parameter Values](#getting-parameter-values)
- [Setting Parameters](#setting-parameters)
- [Describing Parameters](#describing-parameters)
- [Dumping Parameters](#dumping-parameters)
- [Loading Parameters](#loading-parameters)
- [Deleting Parameters](#deleting-parameters)
- [Advanced Usage](#advanced-usage)

---

## Building and Running

### 1. Build the Package
```bash
cd ~/bumperbot_ws
colcon build --packages-select bumperbot_cpp_examples
```

### 2. Source the Workspace
```bash
source install/setup.bash
```

### 3. Run the Node with Initial Parameters
```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args -p simple_int_param:=30
```

You can also set multiple parameters at launch:
```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args \
  -p simple_int_param:=30 \
  -p simple_double_param:=3.14 \
  -p simple_string_param:="Hello ROS2"
```

---

## Listing Parameters

### List All Parameters in the System
```bash
ros2 param list
```

### List Parameters for a Specific Node
```bash
ros2 param list /simple_parameter
```

---

## Getting Parameter Values

### Get a Single Parameter Value
```bash
ros2 param get /simple_parameter simple_int_param
```

### Get All Parameters from a Node
```bash
ros2 param dump /simple_parameter
```

This will display all parameters in YAML format to stdout.

---

## Setting Parameters

### Set a Parameter at Runtime
```bash
# Integer parameter
ros2 param set /simple_parameter simple_int_param 42

# Double parameter
ros2 param set /simple_parameter simple_double_param 2.718

# String parameter
ros2 param set /simple_parameter simple_string_param "Updated value"

# Boolean parameter
ros2 param set /simple_parameter simple_bool_param true
```

### Set Array Parameters
```bash
# Integer array
ros2 param set /simple_parameter int_array "[1, 2, 3, 4, 5]"

# Double array
ros2 param set /simple_parameter double_array "[1.1, 2.2, 3.3]"

# String array
ros2 param set /simple_parameter string_array "['ros2', 'humble', 'jazzy']"
```

---

## Describing Parameters

### Get Information About a Parameter
```bash
ros2 param describe /simple_parameter simple_int_param
```

This shows:
- Parameter name
- Type
- Description
- Constraints (if any)
- Read-only status

### Describe All Parameters of a Node
```bash
ros2 node info /simple_parameter
```

---

## Dumping Parameters

### Save Parameters to a YAML File
```bash
ros2 param dump /simple_parameter --output-dir ./config
```

This creates a YAML file with all current parameter values.

### Save to a Specific File
```bash
ros2 param dump /simple_parameter > my_params.yaml
```

### Print Parameters Without Saving
```bash
ros2 param dump /simple_parameter --print
```

---

## Loading Parameters

### Load Parameters from a YAML File
```bash
ros2 param load /simple_parameter ./config/simple_parameter.yaml
```

### Load Parameters at Node Startup
```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args \
  --params-file ./config/simple_parameter.yaml
```

---

## Deleting Parameters

### Delete a Dynamic Parameter
**Note:** Only dynamically declared parameters can be deleted. Statically declared parameters cannot be deleted at runtime.

```bash
ros2 param delete /simple_parameter dynamic_param_name
```

If the parameter was declared statically (in the node's code), you'll get an error:
```
Setting parameter failed: parameter 'param_name' cannot be deleted
```

---

## Advanced Usage

### Monitor Parameter Changes in Real-Time
```bash
ros2 param list /simple_parameter --include-hidden --no-daemon
```

### Use Parameter Event Callbacks
When you change a parameter, ROS2 nodes can react to these changes through parameter callbacks. Monitor these events:
```bash
ros2 topic echo /parameter_events
```

### Set Parameters with YAML Format
Create a `params.yaml` file:
```yaml
simple_parameter:
  ros__parameters:
    simple_int_param: 100
    simple_double_param: 3.14159
    simple_string_param: "ROS2 Parameters"
    simple_bool_param: true
```

Then load it:
```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args \
  --params-file params.yaml
```

### Using Parameter Wildcards
Load parameters for multiple nodes:
```yaml
/**:
  ros__parameters:
    use_sim_time: true
    
simple_parameter:
  ros__parameters:
    simple_int_param: 50
```

### Remapping Parameter Names
```bash
ros2 run bumperbot_cpp_examples simple_parameter --ros-args \
  -r __params:=/path/to/params.yaml
```

### Set Parameters via Launch Files
In your launch file, you can set parameters:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bumperbot_cpp_examples',
            executable='simple_parameter',
            name='simple_parameter',
            parameters=[{
                'simple_int_param': 30,
                'simple_double_param': 2.718,
                'simple_string_param': 'Launch param'
            }]
        )
    ])
```

---

## Common Use Cases

### 1. Debug Parameter Values
```bash
# Check current value
ros2 param get /simple_parameter simple_int_param

# Change and test
ros2 param set /simple_parameter simple_int_param 999

# Verify change
ros2 param get /simple_parameter simple_int_param
```

### 2. Save Current Configuration
```bash
# Dump all parameters
ros2 param dump /simple_parameter --output-dir ./backup

# Later restore them
ros2 param load /simple_parameter ./backup/simple_parameter.yaml
```

### 3. Parameter Validation
Some parameters have constraints (min/max, allowed values). Setting invalid values will fail:
```bash
ros2 param set /simple_parameter constrained_param 9999
# Error: Value out of range
```

---

## Troubleshooting

### Parameter Not Found
```bash
# List all parameters first
ros2 param list /simple_parameter

# Check if node is running
ros2 node list
```

### Permission Denied
Some parameters are read-only and cannot be changed at runtime.

### Type Mismatch
Ensure you're setting the correct type:
```bash
# Wrong: setting string to int parameter
ros2 param set /simple_parameter simple_int_param "text"  # ERROR

# Correct: setting int value
ros2 param set /simple_parameter simple_int_param 42  # OK
```

---

## Additional Resources

- [ROS2 Parameters Documentation](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
- [ROS2 CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Parameter YAML File Format](https://docs.ros.org/en/humble/How-To-Guides/Parameters-YAML-files-migration-guide.html)