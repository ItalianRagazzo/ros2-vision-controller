# ROS 2 Autonomous Vision-to-Control Pipeline

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv)


## Overview
This repository contains a modular, real-time perception and kinematics pipeline built from scratch in ROS 2 Jazzy using C++. It demonstrates the ability to capture raw hardware sensor data, process it using classical computer vision techniques, and translate that spatial data into physical actuation commands via closed-loop proportional control.

This project was built to showcase a highly modular publisher/subscriber architecture, keeping the vision algorithms completely agnostic to the underlying hardware kinematics.

## System Architecture

The pipeline consists of a 4-node architecture:

1. **Hardware Driver (`v4l2_camera_node`)**: Interfaces directly with the local webcam hardware, publishing raw BGR video frames to `/camera/image_raw`.
2. **Perception Brain (`tracker_node`)**: A custom C++ node utilizing `cv_bridge` and OpenCV. It converts frames to the HSV color space to mitigate lighting variance, applies a binary threshold to isolate a target color (green), and calculates the spatial moments (`m.m10 / m.m00`) to find the exact pixel centroid. It publishes this X/Y coordinate as a `geometry_msgs/Point` to `/target_centroid`.
3. **Kinematics Brain (`control_node`)**: A custom C++ node that subscribes to the spatial coordinates. It calculates the error between the target's current X-coordinate and the camera's center frame, applies a **Proportional Control (P-Controller)** algorithm, and publishes rotational and linear velocity commands (`geometry_msgs/Twist`) to `/turtle1/cmd_vel`.
4. **Actuation/Simulation (`turtlesim_node`)**: A lightweight 2D kinematics simulator that physically executes the Twist commands. *(Because of the modular architecture, this node can be seamlessly swapped out for a physical robot's motor controller without altering the perception code).*

## Prerequisites

* **OS:** Ubuntu 24.04
* **Middleware:** ROS 2 Jazzy
* **Libraries:** OpenCV 4
* **ROS 2 Packages:** `v4l2_camera`, `turtlesim`, `cv_bridge`

## Build Instructions

Clone this package into the `src` directory of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <your-repository-url> target_tracker
cd ~/ros2_ws
colcon build --packages-select target_tracker
source install/setup.bash
