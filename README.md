# Scene Segmenter

![Scene Segmenter](demo.gif "Scene Segmenter")

A framework for real-time segmentation of given images (video frames) based on the given parameters and configurations. The main use case of this repository is to be used in [vS-Graphs](https://github.com/snt-arg/visual_sgraphs), where the camera output seen by the robot is sent to the current package to be segmented.

## 📚 Preparation

### I. Cloning

Create a new workspace and clone the repo in its `src` folder. In case the repository is going to be used for `vS-Graphs`, it is recommended to clone the repository in the same `src` folder where it exists, as `vS-Graphs` depends on it. Accordingly, you can use the command below:

```
git clone --recurse-submodules git@github.com:snt-arg/scene_segment_ros.git
```

After cloning the repository, you may add a command like `alias sourcecsr='source ~/workspace/[PATH]/devel/setup.bash'` in your `.bashrc` file.

### II. Installing Python Libraries

Install the required `Python` libraries for running this program using the command below:

```
pip install -r src/requirements.txt
```

### III. Running the program

**Run with the realsense camera**

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

**Or from a rosbag file**
```bash
roscore
```
```bash
rosbag play <file> --clock
```

**Initialise the main nodes** (the name of the packager may change):

**Segmenter node**
```bash
rosrun segmenter_ros2 segmenter.py
```

**Interpreter node**
```bash
rosrun segmenter_ros2 interpreter.py
```
**To run the visualization** (for demo)
```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
```
```bash
rosrun segmenter_ros2 visualization.py
```
```bash
rosrun rviz rviz
```
In the rviz window, click *add* -> *MarkerArray* and set the *marker topic* to `/visualization`

Then, run the image mapper node to start the program:
```bash
rosrun segmenter_ros2 imageMapper.py
```


## 🔨 Configurations

| Main Category  | Parameter               | Default        | Description                  |
| -------------- | ----------------------- | -------------- | ---------------------------- |
| `image_params` | `image_params`          | 640            | width of the input image     |
| `ros_topics`   | `raw_image_topic`       | `/img`         | raw image topic              |
|                | `segmented_image_topic` | `/seg`         | segmented image topic        |
| `model_params` | `model_name`            | -              | name of the model            |
|                | `model_path`            | -              | path of the model file       |
|                | `point_prompt`          | [[0, 0]]       | a point for segmentation     |
|                | `box_prompt`            | [[0, 0, 0, 0]] | boxes for segmentation       |
|                | `text_prompt`           | -              | text prompt (e.g., "a dog")  |
|                | `point_label`           | [0]            | 0: background, 1: foreground |
|                | `iou`                   | 0.9            | annots filtering threshold   |
|                | `conf`                  | 0.4            | object confidence threshold  |
|                | `contour`               | False          | draw contours                |

## 🚀 Running the Code

How to run the code ....

## 📊 Segmenter Models Benchmarking

Here you can see the benchmarking results of the work in the table below. Some of these libraries are already available in [this repository](https://github.com/snt-arg/scene_segmentation).

| Method / Image Resolution                                                      | 480x360     | 640x480     | 720x540     |
| ------------------------------------------------------------------------------ | ----------- | ----------- | ----------- |
| [Segment Anything (SAM)](https://github.com/facebookresearch/segment-anything) | 6.8 seconds | 6.9 seconds | 6.9 seconds |
| Semantic SAM                                                                   | ~2 minutes  | ~2 minutes  | ~2 minutes  |
| [Segment Any RGBD](https://github.com/Jun-CEN/SegmentAnyRGBD)                  | ~2 minutes  | ~2 minutes  | ~2 minutes  |
| [FAST SAM](https://github.com/CASIA-IVA-Lab/FastSAM)                           | 2.4 seconds | 4.5 seconds | 6.5 seconds |
| Mobile SAM                                                                     | 4.1 seconds | 4.7 seconds | 4.3 seconds |

## 📅 TODO

- Finding a realtime Semantic Segmentation framework
  - Benchmarking and comparing
- Excluding unnecessary segments (done)
- Removing logs
- Providing segment information (done)
- Searching for semantic entities (object detection) (done)
- Merging it with ROS-based ORB-SLAM for mapping
- Testing with a robot
  - Drone/Spot
