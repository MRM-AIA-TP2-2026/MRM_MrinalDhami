from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    group_robot_share = get_package_share_directory('robot2')

    # Declare Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use sim time if true'
    )

    # Path to robot URDF
    robot_description = Command([
        'xacro ', os.path.join(group_robot_share, 'model', 'group_robot.xacro')
    ])

    return LaunchDescription([
        use_sim_time,  # Declare argument

        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
        ),

        # Run robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'group_robot',  # Name of the spawned entity
                '-topic', '/robot_description'
            ],
            output='screen'
        ),
    ])

