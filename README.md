# Segmentor

A framework for real-time segmentation of given images

## 📝 Prerequisites

The required libraries ...

## ⚙️ Installation

Installation of the repository ....

## 🔨 Configurations

The parameters to be set and used ....

## 🚀 Running the Code

How to run the code ....

## 📊 Benchmarking

Here you can see the benchmarking results of the work:

| Method / Image Resolution | 480x360   | 640x480   | 720x540   |
| ------------------------- | --------- | --------- | --------- |
| Segment Anything (SAM)    | 6.8 seconds | 6.9 seconds | 6.9 seconds |
| Semantic SAM              | ~2 minutes | ~2 minutes | ~2 minutes |
| Segment Any RGBD          | ~2 minutes | ~2 minutes | ~2 minutes |
| FAST SAM                  | 2.4 seconds | 4.5 seconds | 6.5 seconds |
| Mobile SAM                | 4.1 seconds | 4.7 seconds | 4.3 seconds |

## 📅 TODO

- Finding a realtime Semantic Segmentation framework
  - Benchmarking and comparing
- Implementation of a `ROS` wrapper (separate repo)
  - Testing
- Merging it with ROS-based ORB-SLAM for mapping
- Testing with a robot
  - Drone/Spot
