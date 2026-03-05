# AshBot Worlds

This package contains the worlds for the AshBot project. The worlds are created
using the [Gazebo](http://gazebosim.org/) simulator and the
[ROS](http://www.ros.org/) framework.

## Worlds

There 3 worlds in this package:

- `empty`: An empty world with a ground plane.
- `guided_maze`: A maze with red and green boxes to help guide the robot to
  the goal.
- `wall_arena`: An arena with walls.

## Installation

Git clone the repository into your workspace by first opening a terminal and navigating to the `src` directory of your workspace. Then run the following command:

```bash
git clone https://github.com/Ashesi-Robotics/ashbot_world.git
```

Next, change directorty to `ashbot_world` and run the following command to
install the package python dependencies:

```bash
pip install -r requirements.txt
```

Next, go to the root of your workspace and run the following command to install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Finally, build the package by running the following command:

```bash
colcon build --packages-select ashbot_world
```

## Usage

To launch a world, run the following command:

```bash
ros2 launch ashbot_world world.launch.py world:=<world_name>
ros2 launch ashbot_world world.launch.py world:=wall_arena
```

where `<world_name>` is the name of the world you want to launch as defined in
the `worlds` above.

For example, to launch the `guided_maze` world, run the following command:

```bash
ros2 launch ashbot_world world.launch.py world:=guided_maze
```

The default world is `empty` and that can be launched by running the following
command:

```bash
ros2 launch ashbot_world world.launch.py
```

### Guided Maze World

In order to regenerate a new maze and launch the `guided_maze` world, run the following command:

```bash
ros2 launch ashbot_world guided_maze.launch.py
```

This also accepts the following launch arguments:

- `width`: The width of the maze. Default is `11`.
- `height`: The height of the maze. Default is `11`.
- `cell_size`: The size of each cell in the maze. Default is `0.5` metres.
- `box_height`: The height of the boxes. Default is `0.4` metres.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE)
file for details.
