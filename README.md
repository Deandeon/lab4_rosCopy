[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/LBOJ36UH)




# Intro To AI robotics Lab 3 Documentation

[![License](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Production%20Ready-success?style=flat-square)]()

 [ Video Tour](#-video-demonstration)

</div>


##  Overview
This documentation Hghlights the structure of this repository. The objective of this lab was  to simulate and create an autonomous Wall-Following Robot in Gazebo via simulations and use those as guidelines for actually running the algoritm on a limo robot.


### Files

<table>
<tr>
<td width="50%">

####  Main Files
- **mobille Robot** - This is the projects description file which contains the robot models 
- **Wall_follower** - Wall fllowing algorithm 
- **robot_bringup** -Bring up file for project assembly and quick launch
- **Ashbot_world** - File containing various world files that the robot can navigate through

</td>
<td width="50%">

####  MISC
- **Dev container** 
- **Controller Files** 
- **unit tests** 

</td>
</tr>
</table>


## File structure
```
─lab-3-locomotion-and-sensing-akastsuki
    │   .gitignore
    │   LICENSE
    │   README.md
    │   ruff.toml
    │
    ├───.devcontainer
    │   │   devcontainer.json
    │   │   Dockerfile
    │   │
    │   └───scripts
    │           delock.sh
    │           kill_ros.sh
    │           shell_setup.sh
    │
    ├───ashbot_world
    │   │   .gitignore
    │   │   .gitmodules
    │   │   CMakeLists.txt
    │   │   LICENSE
    │   │   package.xml
    │   │   README.md
    │   │   requirements.txt
    │   │   ruff.toml
    │   │
    │   ├───hooks
    │   │       ashbot_world.dsv.in
    │   │       ashbot_world.sh.in
    │   │
    │   ├───launch
    │   │       guided_maze.launch.py
    │   │       world.launch.py
    │   │
    │   ├───models
    │   │   ├───ground_plane
    │   │   │       model.config
    │   │   │       model.sdf
    │   │   │
    │   │   └───sun
    │   │           model.config
    │   │           model.sdf
    │   │
    │   ├───scripts
    │   │       guided_maze.py
    │   │       maze.py
    │   │       __init__.py
    │   │
    │   ├───templates
    │   │   │   guided_maze.world.jinja
    │   │   │
    │   │   └───macros
    │   │           common.jinja
    │   │           macros.jinja
    │   │
    │   └───worlds
    │           empty.world
    │           guided_maze.world
    │           wall_arena.world
    │
    ├───mobile_robot
    │   │   CMakeLists.txt
    │   │   package.xml
    │   │
    │   ├───launch
    │   │       gazebo_model.launch.py
    │   │
    │   ├───model
    │   │       robot.gazebo
    │   │       robot.xacro
    │   │
    │   └───worlds
    │           empty_world.world
    │           wall_world.sdf
    │
    ├───robot_bringup
    │   │   CMakeLists.txt
    │   │   package.xml
    │   │   README.md
    │   │
    │   └───launch
    │           bringup.launch.py
    │
    └───wall_follower
        │   package.xml
        │   README.md
        │   setup.cfg
        │   setup.py
        │
        ├───resource
        │       wall_follower
        │
        ├───test
        │       test_copyright.py
        │       test_flake8.py
        │       test_pep257.py
        │       wall_follower_testUnits
        │
        └───wall_follower
                wall_follower.py
                __init__.py
```

<details>
<summary><b> How to simulate</b></summary>

- Open the user terminal
- Ensure you are within a valid ros2 workspace with all the necessary dependacies installed
- run this :
`ros2 launch mobile_robot bringup.launch.py `

this runs the bringup file which assembles the robot for the simulation
</details>
 
## Description Package (mobile_robot)
-The mobile robot package is responsible for the physical simulation of the robot system. it provides the robot model, some world environment test files , and  the simulation launch configuration needed for the Gazebo simulation

it als contains the robot.xarco which holds the URDF files such as the robots links and joints and other coordinate frames. As well the robot.gazebo file which maps the published cmds from thier respective states into actual values for the joints or instructions

## Topics in Mobile robot
- these include : 
`/scan `  - Laser- Lidar
`/cmd_vel`	 -ROS → GZ	Diff-drive plugin
`/odom`	GZ → ROS	Odometry plugin
`/tf`	ROS	robot state publisher
 


## Wall-Following Algorithm(wall_Follower)
The wall-following controller is a state-based control system that uses LiDAR range measurements to determine movement direction. It works by :

- Detecting a wall/obstacle
- Selects a side (left/right)
- Maintains a bounded distance from that wall
- Avoids frontal collisions

NB:
It does not build a map or plan globally.
It operates entirely on instantaneous sensor data


</div>