from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
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
    use_simple_controller_arg = DeclareLaunchArgument(
        'use_simple_controller', default_value='True',
        description='Whether to use the simple controller or not'
    )
    # Read the run-time value of the parameters
    use_python = LaunchConfiguration('use_python') 
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_seperation = LaunchConfiguration('wheel_seperation')
    use_simple_controller = LaunchConfiguration('use_simple_controller')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )
    ############################
    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions=  
        [
            # Spawner for the simple velocity controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            ),
            # Simple Controller Node in Python
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                name="simple_controller",
                parameters=[{
                    'wheel_radius': wheel_radius,
                    'wheel_seperation': wheel_seperation
                }],
                condition=IfCondition(use_python)
            ),
            # Simple Controller Node in C++
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                name="simple_controller",
                parameters=[{
                    'wheel_radius': wheel_radius,
                    'wheel_seperation': wheel_seperation
                }],
                condition=UnlessCondition(use_python)
            )
        ]
    )
    


    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_seperation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller
    ])