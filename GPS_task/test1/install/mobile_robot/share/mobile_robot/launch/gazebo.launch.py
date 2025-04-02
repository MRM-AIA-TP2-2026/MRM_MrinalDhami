import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    robotXacroname = 'mobile_robot'
    packageName = 'mobile_robot'

    modelrelativepath = 'model/mobile_robot.xacro'
    worldrelativepath = 'model/simulate1.world'

    pathModeFile = os.path.join(get_package_share_directory(packageName), modelrelativepath)
    pathWorldFile = os.path.join(get_package_share_directory(packageName), worldrelativepath)

    robotDescription = xacro.process_file(pathModeFile).toxml()
    
    #GAzebo launch file package is defined here
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(package='gazebo_ros',executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', robotXacroname],output='screen')
    
        
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True}]
    )

    launchDescriptionObject = LaunchDescription()
    
    launchDescriptionObject.add_action(gazeboLaunch)
    
    
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    
    return launchDescriptionObject

