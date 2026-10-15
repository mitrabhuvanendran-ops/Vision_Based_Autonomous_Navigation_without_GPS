# Vision-Based Autonomous Navigation for an Outdoor UGV

A ROS 2 project for vision-based navigation of an unmanned ground vehicle in outdoor environments.
It was developed for the Smart India Hackathon and includes a Gazebo simulation, terrain segmentation, costmap generation, path planning, and motion control.
The ROS 2 workspace is under `ugv_ws/`, with the OFFSEG segmentation implementation and related utilities in `OFFSEG-main/`.

## Team

- YOUR NAME ([GitHub](https://github.com/YOUR_USERNAME))
- Udhayan K ([GitHub](https://github.com/Udhayan27-Dev))

## Overview

The simulated RGB camera feeds `bisenet_node`, which publishes a traversability mask. If model weights are not found at the configured paths, the node falls back to a terrain-image heuristic. `costmap_builder` synchronizes that mask with depth data, projects valid points into the `odom` frame, and combines them with laser scan data to publish an occupancy grid.

A goal entered as a bearing and distance is converted from the robot frame into an `odom`-frame pose. `rrt_planner` plans and replans an obstacle-aware RRT* path over the costmap; `path_follower` turns it into velocity commands. `safety_gate` checks the laser scan, IMU slope, and plan freshness before forwarding commands. The simulation launch also starts an EKF and a web dashboard; ORB-SLAM3 is optional.

## Repository Structure

- `ugv_ws/`: Colcon workspace. Build from this directory.
- `ugv_ws/src/ugv_nav/`: BiSeNet inference, costmap, goal conversion, RRT* planner, path follower, safety gate, EKF/ORB-SLAM launch files, and configuration.
- `ugv_ws/src/ugv_robot/`: UGV Xacro model, Gazebo sensor setup, robot launch, RViz configuration, and data-collection/training scripts.
- `ugv_ws/src/ugv_terrain/`: Gazebo outdoor world, models, and materials.
- `ugv_ws/src/ugv_sim/`: Integrated and split-machine simulation bringup launch files.
- `ugv_ws/src/ugv_web/`: Browser dashboard, rosbridge/web-video launch, and static web assets.
- `ugv_ws/src/orbslam3_ros2/` and `ugv_ws/src/ORB_SLAM3/`: Empty in this checkout. The optional ORB-SLAM launch references the `orbslam3_ros2` package and an ORB vocabulary under `ORB_SLAM3`.
- `OFFSEG-main/`: OFFSEG code, BiSeNet models, segmentation pipeline, datasets/utilities, and upstream license/readmes.

## Hardware

The repository defines a Gazebo UGV model with four driven wheels, a 2D ray LiDAR, RGB and depth cameras, and an IMU. The model config specifies a 640x480 RGB camera at 30 Hz, a 640x480 depth camera at 20 Hz, a 720-sample LiDAR at 20 Hz with a 0.15-30 m range, and an IMU at 100 Hz. These are simulation settings, not identification of physical sensor makes or models.

Physical UGV hardware and sensor models: **TODO**.

## Software Requirements

- ROS 2 distribution: **TODO**. The package manifests do not specify a distribution.
- ROS package dependencies include `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, `tf2_ros`, `cv_bridge`, `message_filters`, `robot_localization`, `robot_state_publisher`, `joint_state_publisher`, `xacro`, `gazebo_ros`, `rviz2`, `rosbridge_server`, and `web_video_server`.
- The Python setup files declare `setuptools`; the navigation source also imports NumPy, OpenCV, PyTorch, and SciPy. The ROS package manifests/setup files do not declare those as Python install requirements.
- Python version for the ROS workspace: **TODO**. The root `OFFSEG-main/requirements.txt` pins Python `3.6.13` and includes `libKMCUDA==6.2.3`, `matplotlib==3.3.4`, `numpy==1.19.5`, `opencv_python==4.5.5.62`, `pandas==1.1.5`, `Pillow==9.2.0`, `scikit_learn==1.1.2`, `tensorflow==2.6.2`, `torch==1.9.0+cu111`, `torchvision==0.10.0+cu111`, `tqdm==4.62.3`, and `fast-pytorch-kmeans==0.2.0`.
- The nested HRNet requirements file lists `EasyDict==1.7`, `opencv-python`, `shapely==1.6.4`, `Cython`, `scipy`, `pandas`, `pyyaml`, `json_tricks`, `scikit-image`, `yacs>=0.1.5`, `tensorboardX>=1.6`, `tqdm`, and `ninja`.

The OFFSEG requirement files describe the segmentation subprojects and are not a declaration that all listed versions form a compatible ROS workspace environment.

## Build and Run

Run these commands in a ROS 2 Linux environment with the required ROS packages installed. Replace `<ROS_DISTRO>` with the installed distribution; the repository does not name one.

```bash
cd ugv_ws
source /opt/ros/<ROS_DISTRO>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Launch the integrated Gazebo simulation, EKF, dashboard, and navigation nodes. ORB-SLAM3 is disabled because its source/package files are not present in this checkout:

```bash
ros2 launch ugv_sim full_stack.launch.py use_orbslam:=false
```

Other launch files provided by the source include:

```bash
ros2 launch ugv_robot forest_robot.launch.py
ros2 launch ugv_sim pc_bringup.launch.py
ros2 launch ugv_sim jetson_bringup.launch.py
ros2 launch ugv_web dashboard.launch.py
```

Run an individual installed navigation executable with:

```bash
ros2 run ugv_nav bisenet_node
```

The BiSeNet node looks for weights at `~/ugv_ws/models/bisenet_sim_2class.pth`, then `~/OFFSEG-main/Models/BiSeNet_RUGD/model_final.pth`; without either file it uses its heuristic fallback.

## ROS 2 Nodes and Topics

| Node | Subscribes | Publishes |
| --- | --- | --- |
| `bisenet_node` | `/camera/rgb_camera/image_raw` | `/seg/mask` |
| `costmap_builder` | `/camera/rgb_camera/camera_info`, `/seg/mask`, `/camera/depth/depth/image_raw`, `/scan` | `/costmap` |
| `goal_converter` | `/dashboard/goal_polar` | `/goal_pose`, `/goal_marker` |
| `rrt_planner` | `/costmap`, `/goal_pose` | `/plan` |
| `path_follower` | `/plan` | `/cmd_vel_raw` |
| `safety_gate` | `/scan`, `/imu/data`, `/plan`, `/cmd_vel_raw` | `/cmd_vel` |
| `auto_wander` (standalone robot script) | `/scan` | `/cmd_vel` |

## Acknowledgements

- Udhayan K: [GitHub](https://github.com/Udhayan27-Dev).
- The [OFFSEG project](https://github.com/kasiv008/OFFSEG) and authors Kasi Viswanath, Kartikeya Singh, Peng Jiang, P. B. Sujit, and Srikanth Saripalli for *OFFSEG: A Semantic Segmentation Framework for Off-Road Driving* (2021). The included `OFFSEG-main/` project provides its Apache-2.0 license in `OFFSEG-main/LICENSE.md`.
