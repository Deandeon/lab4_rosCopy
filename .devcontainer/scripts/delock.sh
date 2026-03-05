#
# File: delock.sh
# Project: #LAB_1: Intro_To_ROS2
# File Created: Wednesday, 4th February 2026 8:06:27 PM
# Author: #TODO: Add author name
# Email: gloria.otieno@ashesi.edu.gh
# Version: 1.0.0
# Brief: <<brief>>
# -----
# Last Modified: Wednesday, 4th February 2026 8:06:27 PM
# Modified By: #TODO: Add author name
# -----
# Copyright ©2026 #TODO: Add author name
#
#!/bin/bash

sudo lsof /var/lib/dpkg/lock
sudo lsof /var/lib/dpkg/lock-frontend
sudo lsof /var/lib/apt/lists/lock

sudo rm /var/lib/dpkg/lock
sudo rm /var/lib/dpkg/lock-frontend
sudo rm /var/lib/apt/lists/lock
sudo rm /var/cache/apt/archives/lock

sudo dpkg --configure -a