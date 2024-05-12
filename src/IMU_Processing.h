#pragma once
#include <cmath>
#include <math.h>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)
const bool time_list(PointType &x, PointType &y); // {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  MD(12, 12) state_cov = MD(12, 12)::Identity();
  int    lidar_type;
  V3D    gravity_;
  bool   imu_en;
  V3D    mean_acc;
  bool   imu_need_init_ = true;
  bool   after_imu_init_ = false;
  bool   b_first_frame_ = true;
  double time_last_scan = 0.0;
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001);
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001);

 private:
  void IMU_init(const MeasureGroup &meas, int &N);

  V3D mean_gyr;
  int init_iter_num = 1;
  rclcpp::Logger logger;
};
