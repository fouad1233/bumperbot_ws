from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        'use_python', default_value='True',
        description='Whether to use the Python version of the controller if false cpp will be used'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.033',
        description='Radius of the wheels'
    )
    wheel_seperation_arg = DeclareLaunchArgument(
        'wheel_seperation', default_value='0.17',
        description='Seperation between the wheels'
    )
    # Read the run-time value of the parameters
    use_python = LaunchConfiguration('use_python') 
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_seperation = LaunchConfiguration('wheel_seperation')
    
    simple_controller_py = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        name="simple_controller",
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_seperation': wheel_seperation
        }],
        condition=IfCondition(use_python)
    )
    simple_controller_cpp = Node(
        package="bumperbot_controller",
        executable="simple_controller",
        name="simple_controller",
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_seperation': wheel_seperation
        }],
        condition=UnlessCondition(use_python)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_seperation_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_py,
        simple_controller_cpp
    ])