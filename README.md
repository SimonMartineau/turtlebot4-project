# 2024 TalTech Internship Codes

## Overview
This repository contains two ROS2 packages I designed for the SLAM experiments with the Turtlebot4 robot.

### Packages
1. **Package 1**: [turtlebot4_slam_noise]  
   - [This packages contains the programs that generate visual and LiDAR noise for the experiment. The 2 programs in question are called visual_noise.py and lidar_dust_noise_v4.py respectively. The rest of the codes that are not used in the final experiment are in the milestone_codes folder].
   
2. **Package 2**: [turtlebot4_bringup]  
   - [This package contains the launch files and the configuration files to setup the visual and LiDAR noise experiments.].

## Table of Contents
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [Package 1 Details](#package-1-details)
- [Package 2 Details](#package-2-details)

## Installation
To install and build the packages, follow the steps below:

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/SimonMartineau/turtlebot4-project.git

## Dependencies
To be able to replicate my experiments, here are the 2 programs that you need to install:

1. Install NAV2:

   Follow the installation guide here: https://docs.nav2.org/getting_started/index.html#installation
   
2. Install RTAB-Map:

   Follow the installation guide here: https://github.com/introlab/rtabmap/wiki/Installation
   
3. You will also need to modify one aspect of these programs to make them work together. In NAV2, you will need to change /scan to /modified_scan. For RTAB-Map, you will need to change image_raw to image_modified.

## Usage

1. In turtlebot4_bringup/config, setup the visual and LiDAR noise by modifying the yaml files.

2. Build and source your ROS2 workspace if you haven't done so. 

3. Run the all_noise.launch.py file for example.

4. Run NAV2 or RTAB-Map with start_nav2.launch.py or start_rtabmap.launch.py.

## 





    
