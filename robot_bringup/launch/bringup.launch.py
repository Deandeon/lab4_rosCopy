import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'worlds/wall_world.sdf' #CHANGE THIS TO THE ACTUAL WORLD NAME 
    worldPackage = 'mobile_robot'
    pathModelFile = os.path.join(get_package_share_directory(
        namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(
        worldPackage), worldFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, launch_arguments={'gz_args': f'-r {pathWorldFile}'}.items())
    spawnModelNode = Node(package='ros_gz_sim', executable='create', arguments=[
                          '-name', robotXacroName, '-topic', 'robot_description'], output='screen')

    
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robotDescription, 'use_sim_time': True}]
    )

    bridgeCmdVel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )

    bridgeOdom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    bridgeScan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    wallFollowerNode = Node(
        package='wall_follower',
        executable='wall_follower',
        output = 'screen'

    )
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(bridgeCmdVel)
    launchDescriptionObject.add_action(bridgeOdom)
    launchDescriptionObject.add_action(bridgeScan)
    launchDescriptionObject.add_action(wallFollowerNode)
    return launchDescriptionObject
