# Point-LIO
## Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry

## 1. Introduction

<div align="center">
    <div align="center">
        <img src="https://github.com/hku-mars/Point-LIO/raw/master/image/toc4.png" width = 75% >
    </div>
    <font color=#a0a0a0 size=2>The framework and key points of the Point-LIO.</font>
</div>

**Point-LIO** is a robust and high-bandwidth LiDAR-inertial odometry with the capability to estimate extremely aggressive robotic motions. Point-LIO has two key novelties that enable a high-bandwidth LiDAR-inertial odometry (LIO). The first one is a point-by-point LIO framework, where the state is updated at each LiDAR point measurement without accumulating them into a frame. This point-by-point update allows an extremely high-frequency odometry output, significantly increases the odometry bandwidth, and also fundamentally removes the artificial in-frame motion distortion in aggressive motions. The second main novelty is a stochastic process-augmented kinematic model which models the IMU measurements as an output, instead of input as in existing filter-based odometry or SLAM systems, of the model. This new modeling method enables accurate localization and reliable mapping for aggressive motions even when IMU measurements are saturated. In experiments, Point-LIO is able to provide accurate, high-frequency odometry (4-8 kHz) and reliable mapping under severe vibrations and aggressive motions with high angular velocity (75 rad s^{-1}) beyond the IMU measuring ranges. And Point-LIO is computationally efficient, robust, versatile on public datasets with general motions. As an odometry, Point-LIO could be used in various autonomous tasks, such as trajectory planning, control, and perception, especially in cases involving very fast ego-motions (e.g., in the presence of severe vibration and high angular or linear velocity) or requiring high-rate odometry output and mapping (e.g., for high-rate feedback control and perception).

## **1.1. Developers:**
The codes of this repo are contributed by:
[Dongjiao He (贺东娇)](https://github.com/Joanna-HE) and [Wei Xu (徐威)](https://github.com/XW-HKU)


## **1.2. Related paper**
Our paper is accepted to Advanced Intelligent Systems(AIS), and is in the process of production. The preprint version is attached here, [Point-LIO_preprint.pdf](https://github.com/hku-mars/Point-LIO/files/10989136/Point-LIO_preprint.pdf)


## **1.3. Related video**
Our accompany video is available on **YouTube**.
<div align="center">
    <a href="https://youtu.be/oS83xUs42Uw" target="_blank"><img src="https://github.com/hku-mars/Point-LIO/raw/master/image/final.png" width=60% /></a>
</div>

## 2. What can Point-LIO do?
### 2.1 Simultaneous LiDAR localization and mapping (SLAM) without motion distortion

### 2.2 Produce high odometry output frequence and high bandwidth

### 2.3 SLAM with aggressive motions even the IMU is saturated

# **3. Prerequisites**

## **3.1 Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu18.04 with ros melodic and Ubuntu20.04 with noetic. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **3.2 Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```

## **3.3 livox_ros_driver2**
Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

*Remarks:*
- Since the Point-LIO supports Livox serials LiDAR, so the **livox_ros_driver2** must be installed and **sourced** before run any Point-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Licox_ros_driver2_dir$/install/setup.sh ``` to the end of file ``` ~/.bashrc ```, where ``` $Livox_ros_driver2_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

## 4. Build
Clone the repository and colcon_build:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/Point-LIO.git
    cd Point-LIO
    cd ../..
    colcon build --symlink-install
    source install/setup.sh
```
- Remember to source the livox_ros_driver2 before build (follow 3.3 **livox_ros_driver2**)
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```

## 5. Directly run
Important notes:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. Please obtain the saturation values of your used IMU (i.e., accelerator and gyroscope), and the units of the accelerator of your used IMU, then modify the .yaml file according to those settings. That's improtant.

C. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important because Point-LIO processes at the sampling time of each LiDAR point.

D. We recommend to set the **extrinsic_est_en** to false if the extrinsic is give. As for the extrinsic initiallization, please refer to our recent work: [**Robust and Online LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

E. If a high odometry output frequency without downsample is required, set ``` publish_odometry_without_downsample ``` as true. Then the warning message of tf "TF_REPEATED_DATA" will pop up in the terminal window, because the time interval between two publish odometery is too small. The following command could be used to suppress this warning to a smaller frequency:

in your ros_ws/src,

git clone --branch throttle-tf-repeated-data-error git@github.com:BadgerTechnologies/geometry2.git

Then rebuild, source setup.bash, run and then it should be reduced down to once every 10 seconds. If 10 seconds is still too much log output then change the ros::Duration(10.0) to 10000 seconds or whatever you like.

### 5.1 For Mid360

Connect to your PC to Livox Mid360 LiDAR by following  [Livox-ros-driver2 installation](https://github.com/Livox-SDK/livox_ros_driver2), then

```sh
    cd ~/$Point_LIO_ROS_DIR$
    source devel/setup.sh
    ros2 launch point_lio mapping_mid360.launch.py
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

- For livox serials, Point-LIO only support the data collected by the ``` msg_MID360_launch.py ``` since only its ``` livox_ros_driver2/msg/custom_msg ``` data structure produces the timestamp of each LiDAR point which is very important for Point-LIO. ``` rviz_MID360_launch.py ``` can not produce it right now.

- If you want to change the frame rate, please modify the **publish_freq** parameter in the [msg_MID360_launch.py](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/launch_ROS2/msg_MID360_launch.py) of [Livox-ros-driver2](https://github.com/Livox-SDK/livox_ros_driver2) before make the livox_ros_driver2 pakage.

### 5.2 For Avia

As avia's Livox driver is `livox_ros2_driver`, not `livox_ros_driver2`, you need to do as follows.

1. Follow the [Livox official tutorial](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/README.md) to install `livox_ros2_driver` first.
2. Then, you will need to modify the content in the `point_lio` source code by replacing `livox_ros_driver2::msg::CustomMsg` with `livox_ros2_driver::msg::CustomMsg` and replacing `<livox_ros_driver2/msg/custom_msg.hpp>` with `<livox_ros2_driver/msg/custom_msg.hpp>`.

Connect to your PC to Livox Avia LiDAR by following  [Livox-ros2-driver installation](https://github.com/Livox-SDK/livox_ros2_driver), then

```sh
    cd ~/$Point_LIO_ROS_DIR$
    source install/setup.sh
    ros2 launch point_lio mapping_avia.launch.py
    ros2 launch livox_ros2_driver livox_lidar_msg_launch.py
```

### 5.3 For Livox serials with external IMU

mapping_avia.launch theratically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters befor run:

Edit ``` config/avia.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ```
3. Translational extrinsic: ``` extrinsic_T ```
4. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in Point-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.
5. Saturation value of IMU's accelerator and gyroscope: ```satu_acc```, ```satu_gyro```
6. The norm of IMU's acceleration according to unit of acceleration messages: ``` acc_norm ```

### 5.4 For Velodyne or Ouster (Velodyne as an example)

Step A: Setup before run

Edit ``` config/velodyne.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ``` (both internal and external, 6-aixes or 9-axies are fine)
3. Set the parameter ```timestamp_unit``` based on the unit of **time** (Velodyne) or **t** (Ouster) field in PoindCloud2 rostopic
4. Line number (we tested 16, 32 and 64 line, but not tested 128 or above): ``` scan_line ```
5. Translational extrinsic: ``` extrinsic_T ```
6. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in Point-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).
7. Saturation value of IMU's accelerator and gyroscope: ```satu_acc```, ```satu_gyro```
8. The norm of IMU's acceleration according to unit of acceleration messages: ``` acc_norm ```

Step B: Run below

```sh
    cd ~/$Point_LIO_ROS_DIR$
    source install/setup.sh
    ros2 launch point_lio mapping_velody16.launch.py
```

Step C: Run LiDAR's ros driver or play rosbag.

### 5.5 PCD file save

Set ``` pcd_save_enable ``` in launchfile to ``` 1 ```. All the scans (in global frame) will be accumulated and saved to the file ``` Point-LIO/PCD/scans.pcd ``` after the Point-LIO is terminated. ```pcl_viewer scans.pcd``` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 

```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

# **6. Examples**

## **6.1. Example-1: SLAM on datasets with aggressive motions where IMU is saturated**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example1.gif"  width="40%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example2.gif"  width="54%" />
</div>

## **6.2. Example-2: Application on FPV and PULSAR**
<div align="center">
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example3.gif"  width="58%" />
<img src="https://github.com/hku-mars/Point-LIO/raw/master/image/example4.gif"  width="35%" />
</div>

PULSAR is a self-rotating actuated by only one motor, [PULSAR](https://github.com/hku-mars/PULSAR)

## 7. Contact us
If you have any questions about this work, please feel free to contact me <hdj65822ATconnect.hku.hk> and Dr. Fu Zhang <fuzhangAThku.hk> via email.
