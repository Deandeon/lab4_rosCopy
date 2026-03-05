"""
File: world.launch.py
Project: AshBot World
File Created: Monday, 27th January 2025 11:55:37 AM
Author: nknab
Email: kojo.anyinam-boateng@alumni.ashesi.edu.gh
Version: 1.0
Brief: Launch file to launch a world in Gazebo
-----
Last Modified: Monday, 27th January 2025 3:02:13 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = "ashbot_world"


def launch_setup(context: LaunchContext) -> list:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context object to get the launch configuration

    Returns
    -------
    list
        The list of launch nodes to execute

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    world = LaunchConfiguration("world").perform(context)

    world_filepath = join(pkg_share, "worlds", f"{world}.world")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_filepath],
            "on_exit_shutdown": "true",
        }.items(),
    )

    return [gazebo]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to launch a world.

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="empty",
                choices=["empty", "guided_maze", "wall_arena"],
                description="The world to launch",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
