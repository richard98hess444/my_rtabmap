# 3D Mapping with RTAB-Map and Realsense

## Installation

#### 1. Install Realsense
Please follow the official [website](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu) installation.

#### 2. Install RTAB-Map
Original [website](https://github.com/introlab/rtabmap_ros/tree/ros2).
```
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```
#### 3. Install this package
Navigate to your ROS2 workspace and git clone the repository with
```
git clone https://github.com/richard98hess444/my_rtabmap.git
```

## Usage
#### Step 1.
Launch the realsense. 
```
ros2 launch my_rtabmap rs_launch.py
```
This launch file is equivalent to
```
ros2 launch realsense2_camera rs_launch.py \
align_depth:=true \
rgb_camera.color_profile:=640x480x30 \
depth_module.depth_profile:=640x480x30 \
enable_gyro:=true \
enable_accel:=true \
unite_imu_method:=2
```
#### Step 2.
Launch the RTAB-Map
```
ros2 launch my_rtabmap realsense_d400.launch.py
```
This launch file is equivalent to
```
ros2 launch rtabmap_examples realsense_d400.launch.py \
rtabmap_args:="--delete_db_on_start" \
rgb_topic:=/camera/camera/color/image_raw \
depth_topic:=/camera/camera/depth/image_rect_raw \
camera_info_topic:=/camera/camera/color/camera_info \
approx_sync:=false
```
#### Step 3.
The rtabmap-rviz will launch up, and the point cloud could be seen from Rviz2. To save point clouds into `.ply` file, run
```
ros2 run my_rtabmap pointcloud_saver
```
The point cloud will be saved in `ply_data` folder.