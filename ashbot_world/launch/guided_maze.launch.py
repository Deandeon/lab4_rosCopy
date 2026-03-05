"""
File: guided_maze.launch.py
Project: AshBot World
File Created: Sunday, 26th January 2025 6:24:04 PM
Author: nknab
Email: kojo.anyinam-boateng@alumni.ashesi.edu.gh
Version: 1.0
Brief: Launch file to launch the guided maze world
-----
Last Modified: Monday, 27th January 2025 12:02:07 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join
from sys import path

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
WORLD = "guided_maze.world"


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
    width = int(LaunchConfiguration("width").perform(context))
    height = int(LaunchConfiguration("height").perform(context))
    cell_size = float(LaunchConfiguration("cell_size").perform(context))
    box_height = float(LaunchConfiguration("box_height").perform(context))

    # Check if width and height are odd numbers
    if width % 2 == 0 or height % 2 == 0:
        raise ValueError("Width and height must be odd numbers")

    try:
        # Generate the maze
        path.append(join(pkg_share, "scripts"))

        from guided_maze import generate_maze

        generate_maze(width, height, cell_size, box_height)
    except ImportError as e:
        raise ImportError(f"Could not import the generate_maze function {e}") from e

    # Gazebo launch file
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "world.launch.py")]),
        launch_arguments={
            "world": "guided_maze",
        }.items(),
    )

    return [world]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to launch the guided maze world

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "width",
                default_value="11",
                description="The width of the maze, must be an odd number",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="11",
                description="The height of the maze, must be an odd number",
            ),
            DeclareLaunchArgument(
                "cell_size",
                default_value="0.5",
                description="The size of each cell in the maze in metres",
            ),
            DeclareLaunchArgument(
                "box_height",
                default_value="0.4",
                description="The height of the boxes in the maze in metres",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
