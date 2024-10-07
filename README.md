# 2024 TalTech Internship Codes
 
## Overview
This repository contains two ROS2 packages I designed for the SLAM experiments with the Turtlebot4 robot. These packages introduce noise for testing the robustness of SLAM algorithms.

### Packages
1. **Package 1**: [turtlebot4_slam_noise]  
   - This package contains the programs that generate visual and LiDAR noise for the experiment. The two main programs are:
     - visual_noise.py: Introduces visual noise to the camera data.
     - lidar_dust_noise_v4.py: Adds noise to the LiDAR data.
   - Additional programs that were not part of the final experiment are stored in the 'milestone_codes' directory.
   
2. **Package 2**: [turtlebot4_bringup]  
   - This package contains the launch and configuration files required to setup the visual and LiDAR noise experiments.

## Table of Contents
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Usage](#usage)

## Installation
To install and build the packages, follow these steps:

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/SimonMartineau/turtlebot4-project.git
   ```

## Dependencies
Here's how to install the packages and get the code working:

1. **NAV2**:  
   Follow the installation guide [here](https://turtlebot.github.io/turtlebot4-user-manual/software/overview.html).

2. **RTAB-Map**:  
   Follow the installation guide [here](https://github.com/introlab/rtabmap/wiki/Installation).

3. **Modifications**:  
   - For NAV2, modify the following file:
     - **File**: `turtlebot4/turtlebot4_navigation/config/slam.yaml`
     - **Change**: On line 16, replace:
       ```python
       'scan_topic: scan',
       ```
       with:
       ```python
       'scan_topic: scan_modified',
       ```
       This change ensures NAV2 subscribes to the topic with the noisy LiDAR data.
       
   - For RTAB-Map, modify the following file:
     - **File**: `install/rtabmap_launch/share/rtabmap_launch/launch/rtabmap_bringup.launch.py`
     - **Change**: On line 57, replace:
       ```python
       'rgb_topic': '/camera/pi3/color/image_raw',
       ```
       with:
       ```python
       'rgb_topic': '/image_modified',
       ```
       This change ensures RTAB-Map subscribes to the topic with the noisy image data.

## Usage

1. **Configure Noise Settings**:  
   In the `turtlebot4_bringup/config` folder, modify the `.yaml` files to adjust the visual and LiDAR noise settings according to your experiment requirements.

2. **Build and Source Workspace**:  
   Ensure that your ROS 2 workspace is built and sourced:
   ```bash
   colcon build
   source ~/ros2_ws/install/setup.bash

3. **Run Noise Experiments**:  
   You can run the noise experiments using the launch file:
   ```bash
   ros2 launch turtlebot4_bringup all_noise.launch.py


4. **Run NAV2 or RTAB-Map**:  
   After the noise experiments are running, you can launch either NAV2 or RTAB-Map:
   - For NAV2:
     ```bash
     ros2 launch turtlebot4_bringup start_nav2.launch.py
     ```
   - For RTAB-Map:
     ```bash
     ros2 launch turtlebot4_bringup start_rtabmap.launch.py
     ```









    
