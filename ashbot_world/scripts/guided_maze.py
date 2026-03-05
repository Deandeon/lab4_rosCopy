"""
File: guided_maze.py
Project: AshBot World
File Created: Sunday, 26th January 2025 5:50:10 PM
Author: nknab
Email: kojo.anyinam-boateng@alumni.ashesi.edu.gh
Version: 1.0
Brief: A module for generating a guided maze.
-----
Last Modified: Monday, 27th January 2025 11:50:39 AM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

import itertools
import xml.dom.minidom
from datetime import datetime
from os.path import join
from uuid import uuid4

from jinja2 import Environment, FileSystemLoader
from launch_ros.substitutions import FindPackageShare
from maze import Maze
from PIL import Image, ImageDraw

PACKAGE_NAME = "ashbot_world"
TEMPLATE_DIR = "templates"
TEMPLATE = "guided_maze.world.jinja"
WORLD_DIR = "worlds"


def get_box_placement(
    maze: list[list[str]], solution_path: list[tuple[int, int]]
) -> tuple[list[list[str]], list[tuple[int, int]], list[tuple[int, int]]]:
    """
    Get the placement of the red and green boxes in the maze.

    Parameters
    ----------
    maze : list[list[str]]
        The maze.

    solution_path : list[tuple[int, int]]
        The solution of the maze

    Returns
    -------
    tuple[list[list[str]], list[tuple[int, int]], list[tuple[int, int]]]
        The maze with the boxes placed, the red boxes, and the green boxes.

    """

    directions, red_boxes, green_boxes = [], [], []
    direction_map = {
        ("right", "up"): ("R", -1, -1),
        ("right", "down"): ("L", -1, 1),
        ("left", "up"): ("L", 1, -1),
        ("left", "down"): ("R", 1, 1),
        ("up", "right"): ("L", 1, 1),
        ("up", "left"): ("R", -1, 1),
        ("down", "right"): ("R", 1, -1),
        ("down", "left"): ("L", -1, -1),
    }

    for i in range(1, len(solution_path)):
        dx, dy = (
            solution_path[i][0] - solution_path[i - 1][0],
            solution_path[i][1] - solution_path[i - 1][1],
        )
        directions.append(
            "right" if dx == 1 else "left" if dx == -1 else "down" if dy == 1 else "up"
        )

        if len(directions) > 1:
            prev_direction, current_direction = directions[-2], directions[-1]
            if (current_direction, prev_direction) in direction_map:
                box_type, offset_x, offset_y = direction_map[
                    (current_direction, prev_direction)
                ]
                x, y = solution_path[i][0] + offset_x, solution_path[i][1] + offset_y
                maze[y][x] = box_type
                (red_boxes if box_type == "R" else green_boxes).append((x, y))

    return maze, red_boxes, green_boxes


def convert_to_center_coordinates(
    boxes: list[tuple[int, int]], maze_width: int, maze_height: int
) -> list[tuple[int, int]]:
    """
    Convert the box coordinates to center coordinates.

    Parameters
    ----------
    boxes : list[tuple[int, int]]
        The box coordinates.

    maze_width : int
        The width of the maze.

    maze_height : int
        The height of the maze.

    Returns
    -------
    list[tuple[int, int]]
        The center coordinates of the boxes.

    """

    center_x, center_y = maze_width // 2, maze_height // 2
    return [(center_y - y, center_x - x) for x, y in boxes]


def draw_maze(
    maze: list[list[str]], draw: ImageDraw.ImageDraw, cell_size: int, solution_path=None
) -> None:
    """
    Draw the maze on an image.

    Parameters
    ----------
    maze : list[list[str]]
        The maze.

    draw : ImageDraw.ImageDraw
        The ImageDraw object.

    cell_size : int
        The size of each cell.

    solution_path : list[tuple[int, int]]
        The solution path of the maze.

    """

    color_map = {
        "#": (0, 0, 0),
        "L": (0, 255, 0),
        "R": (255, 0, 0),
        " ": (255, 255, 255),
    }
    height, width = len(maze), len(maze[0])

    for y, x in itertools.product(range(height), range(width)):
        color = color_map.get(maze[y][x], (255, 255, 255))
        if (x, y) == (1, 1):
            color = (255, 255, 0)  # Yellow for the starting cell
        elif (x, y) in [
            (width - 1, height - 2),
            (width - 2, height - 1),
            (width - 1, height - 1),
        ]:
            color = (0, 0, 255)  # Blue for the end cell

        draw.rectangle(
            [x * cell_size, y * cell_size, (x + 1) * cell_size, (y + 1) * cell_size],
            fill=color,
        )

    if solution_path:
        for x, y in solution_path:
            if (x, y) != (1, 1):
                draw.rectangle(
                    [
                        x * cell_size,
                        y * cell_size,
                        (x + 1) * cell_size,
                        (y + 1) * cell_size,
                    ],
                    fill=(128, 128, 128),
                )


def save_maze_to_image(
    maze: list[list[str]], filename: str, cell_size: int = 50
) -> None:
    """
    Save the maze to an image.

    Parameters
    ----------
    maze : list[list[str]]
        The maze.

    filename : str
        The filename to save the image.

    cell_size : int
        The size of each cell.

    """
    height, width = len(maze), len(maze[0])
    img = Image.new("RGB", (width * cell_size, height * cell_size), "white")
    draw = ImageDraw.Draw(img)
    draw_maze(maze, draw, cell_size)
    img.save(filename)


def save_maze_with_solution(
    maze: list[list[str]],
    solution_path: list[tuple[int, int]],
    filename: str,
    cell_size: int = 50,
) -> None:
    """
    Save the maze with the solution path to an image.

    Parameters
    ----------
    maze : list[list[str]]
        The maze.

    solution_path : list[tuple[int, int]]
        The solution path of the maze.

    filename : str
        The filename to save the image.

    cell_size : int
        The size of each cell.

    """
    height, width = len(maze), len(maze[0])
    img = Image.new("RGB", (width * cell_size, height * cell_size), "white")
    draw = ImageDraw.Draw(img)
    draw_maze(maze, draw, cell_size, solution_path)
    img.save(filename)


def generate_maze(
    width: int, height: int, cell_size: float = 0.5, box_height: float = 0.4
) -> None:
    """
    Generate a maze using the recursive backtracking algorithm.

    Parameters
    ----------
    width : int
        The width of the maze.

    height : int
        The height of the maze.

    """
    maze = Maze(width, height)
    maze.generate_maze()

    solution_path = maze.solve_maze()
    maze = maze.get_maze()

    maze, red_boxes, green_boxes = get_box_placement(maze, solution_path)

    red_boxes = convert_to_center_coordinates(red_boxes, width, height)
    green_boxes = convert_to_center_coordinates(green_boxes, width, height)

    blue_boxes = convert_to_center_coordinates(
        [(width - 2, height - 1), (width - 1, height - 2), (width - 1, height - 1)],
        width,
        height,
    )

    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    template_folder = join(pkg_share, TEMPLATE_DIR)
    world_folder = join(pkg_share, WORLD_DIR)

    world_filepath = join(world_folder, TEMPLATE.replace(".jinja", ""))

    env = Environment(loader=FileSystemLoader(template_folder))
    template = env.get_template(TEMPLATE)

    save_maze_to_image(maze, join(pkg_share, "maze.png"))
    save_maze_with_solution(maze, solution_path, join(pkg_share, "maze_solution.png"))

    context = {
        "world_name": "guided maze",
        "generation_date": datetime.now().strftime("%d-%m-%Y %H:%M:%S"),
        "generation_id": str(uuid4()),
        "box_height": box_height,
        "cell_size": cell_size,
        "red_boxes": red_boxes,
        "green_boxes": green_boxes,
        "blue_boxes": blue_boxes,
    }

    world = template.render(context)
    dom = xml.dom.minidom.parseString(world)
    pretty_arena = dom.toprettyxml(indent="  ")

    with open(world_filepath, "w", encoding="utf-8") as f:
        f.write("\n".join(line for line in pretty_arena.splitlines() if line.strip()))
