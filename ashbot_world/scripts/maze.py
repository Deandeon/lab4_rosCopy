"""
File: maze.py
Project: AshBot World
File Created: Sunday, 26th January 2025 5:53:52 PM
Author: nknab
Email: kojo.anyinam-boateng@alumni.ashesi.edu.gh
Version: 1.0
Brief: A module for generating and solving mazes.
-----
Last Modified: Sunday, 26th January 2025 6:00:44 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

import random


class Maze:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

        self.maze = []

    def get_maze(self) -> list[list[str]]:
        """
        Return the maze.

        """
        return self.maze

    def generate_maze(self) -> None:
        """
        Generate a maze using the recursive backtracking algorithm.

        """
        self.maze = [["#"] * self.width for _ in range(self.height)]
        stack = [(1, 1)]
        self.maze[1][1] = " "

        while stack:
            x, y = stack[-1]
            neighbors = [
                (nx, ny)
                for dx, dy in [(-2, 0), (2, 0), (0, -2), (0, 2)]
                if 1 <= (nx := x + dx) < self.width - 1
                and 1 <= (ny := y + dy) < self.height - 1
                and self.maze[ny][nx] == "#"
            ]

            if neighbors:
                nx, ny = random.choice(neighbors)
                self.maze[(y + ny) // 2][(x + nx) // 2] = " "
                self.maze[ny][nx] = " "
                stack.append((nx, ny))
            else:
                stack.pop()

    def solve_maze(self) -> list[tuple[int, int]]:
        """
        Solve the maze using the depth-first search algorithm.

        """

        start, end = (1, 1), (self.width - 2, self.height - 2)
        stack, visited, path = [start], set(), {}

        while stack:
            x, y = stack.pop()
            if (x, y) == end:
                break
            visited.add((x, y))

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if (
                    0 <= nx < self.width
                    and 0 <= ny < self.height
                    and self.maze[ny][nx] == " "
                    and (nx, ny) not in visited
                ):
                    stack.append((nx, ny))
                    path[(nx, ny)] = (x, y)

        solution_path = []
        cell = end
        while cell != start:
            solution_path.append(cell)
            cell = path[cell]
        solution_path.append(start)
        solution_path.reverse()

        return solution_path
