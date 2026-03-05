#
# File: kill_ros.sh
# Project: #LAB_1: Intro_To_ROS2
# File Created: Wednesday, 4th February 2026 8:06:33 PM
# Author: #TODO: Add author name
# Email: gloria.otieno@ashesi.edu.gh
# Version: 1.0.0
# Brief: <<brief>>
# -----
# Last Modified: Wednesday, 4th February 2026 8:06:34 PM
# Modified By: #TODO: Add author name
# -----
# Copyright ©2026 #TODO: Add author name
#
#!/bin/bash

ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
echo "All ROS Processes Killed"

ps aux | grep gazebo | grep -v grep | awk '{ print "kill -9", $2 }' | sh
ps aux | grep gz | grep -v grep | awk '{ print "kill -9", $2 }' | sh
echo "All Gazebo Processes Killed"
