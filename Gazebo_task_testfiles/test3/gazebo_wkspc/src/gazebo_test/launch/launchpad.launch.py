import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotXacroName = 'simple_robot'
    
    namePackage = 'gazebo_test'
    
    modelFileRelativePath = 'model/robot.xacro'
    
    worldFileRelativePath = 'model/simulate1.world'
    
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # This is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    # Here I am defining the launch description
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments={'world': pathWorldFile}.items()
    )
    
    # Now here I will create a gazebo ros Node and the Robot State Publisher
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )
    
    robotStatePublisherNode = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazeboLaunch,
        robotStatePublisherNode,
        spawnModelNode,
    ])

