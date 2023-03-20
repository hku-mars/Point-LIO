# Point-LIO
## Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry

## 1. Introduction
**Point-LIO** is a robust and high-bandwidth LiDAR-inertial odometry with the capability to estimate extremely aggressive robotic motions. Point-LIO has two key novelties that enable a high-bandwidth LiDAR-inertial odometry (LIO). The first one is a point-by-point LIO framework, where the state is updated at each LiDAR point measurement without accumulating them into a frame. This point-by-point update allows an extremely high-frequency odometry output, significantly increases the odometry bandwidth, and also fundamentally removes the artificial in-frame motion distortion in aggressive motions. The second main novelty is a stochastic process-augmented kinematic model which models the IMU measurements as an output, instead of input as in existing filter-based odometry or SLAM systems, of the model. This new modeling method enables accurate localization and reliable mapping for aggressive motions even when IMU measurements are saturated. In experiments, Point-LIO is able to provide accurate, high-frequency odometry (4-8 kHz) and reliable mapping under severe vibrations and aggressive motions with high angular velocity (75 rad s$^{-1}$) beyond the IMU measuring ranges. And Point-LIO is computationally efficient, robust, versatile on public datasets with general motions. As an odometry, Point-LIO could be used in various autonomous tasks, such as trajectory planning, control, and perception, especially in cases involving very fast ego-motions (e.g., in the presence of severe vibration and high angular or linear velocity) or requiring high-rate odometry output and mapping (e.g., for high-rate feedback control and perception).

## **Date of code release**
**The code of Point-LIO will be released before March 20, 2023**.

### 1.1 Related paper
The related paper is accepted to Advanced Intelligent Systems(AIS), and is in the process of production. The preprint version is attached here, [Point-LIO_preprint.pdf](https://github.com/hku-mars/Point-LIO/files/10989136/Point-LIO_preprint.pdf)


### 1.2 Our accompanying videos
Our accompany videos are available on [youtube](https://youtu.be/oS83xUs42Uw)

## 2. What can Point-LIO do?
### 2.1 Simultaneous LiDAR localization and mapping (SLAM) without motion distortion

### 2.2 Produce high odometry output frequence and high bandwidth

### 2.3 SLAM with aggressive motions even the IMU is saturated

## 3. Contact us
If you have any questions about this work, please feel free to contact me <hdj65822ATconnect.hku.hk> and Dr. Fu Zhang <fuzhangAThku.hk> via email.
