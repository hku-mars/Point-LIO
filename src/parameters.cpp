#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;

std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
int  init_map_size, con_frame_num;
double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
float  plane_thr;
double filter_size_surf_min, filter_size_map_min, fov_deg;
double cube_len; 
float  DET_RANGE;
bool   imu_en, gravity_align, non_station_start;
double imu_time_inte;
double laser_point_cov, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT;
std::vector<double> extrinR;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
double time_lag_imu_to_lidar = 0.0;

void readParameters(rclcpp::Node &nh)
{
  p_pre.reset(new Preprocess());

    this->declare_parameter<bool>("prop_at_freq_of_imu", true);
    this->declare_parameter<bool>("use_imu_as_input", true);
    this->declare_parameter<bool>("check_satu", true);
    this->declare_parameter<int>("init_map_size", 100);
    this->declare_parameter<bool>("space_down_sample", true);
    this->declare_parameter<double>("mapping.satu_acc", 3.0);
    this->declare_parameter<double>("mapping.satu_gyro", 35.0);
    this->declare_parameter<double>("mapping.acc_norm", 1.0);
    this->declare_parameter<float>("mapping.plane_thr", 0.05f);
    this->declare_parameter<int>("point_filter_num", 2);
    this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    this->declare_parameter<bool>("common.con_frame", false);
    this->declare_parameter<int>("common.con_frame_num", 1);
    this->declare_parameter<bool>("common.cut_frame", false);
    this->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
    this->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.0);
    this->declare_parameter<double>("filter_size_surf", 0.5);
    this->declare_parameter<double>("filter_size_map", 0.5);
    this->declare_parameter<double>("cube_side_length", 200);
    this->declare_parameter<float>("mapping.det_range", 300.f);
    this->declare_parameter<double>("mapping.fov_degree", 180);
    this->declare_parameter<bool>("mapping.imu_en", true);
    this->declare_parameter<bool>("mapping.start_in_aggressive_motion", false);
    this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    this->declare_parameter<double>("mapping.imu_time_inte", 0.005);
    this->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    this->declare_parameter<double>("mapping.acc_cov_input", 0.1);
    this->declare_parameter<double>("mapping.vel_cov", 20);
    this->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
    this->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    this->declare_parameter<double>("mapping.acc_cov_output", 0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    this->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    this->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    this->declare_parameter<double>("preprocess.blind", 1.0);
    this->declare_parameter<int>("preprocess.lidar_type", 1);
    this->declare_parameter<int>("preprocess.scan_line", 16);
    this->declare_parameter<int>("preprocess.scan_rate", 10);
    this->declare_parameter<int>("preprocess.timestamp_unit", 1);
    this->declare_parameter<double>("mapping.match_s", 81);
    this->declare_parameter<bool>("mapping.gravity_align", true);
    this->declare_parameter<std::vector<double>>("mapping.gravity", {});
    this->declare_parameter<std::vector<double>>("mapping.gravity_init", {});
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_T", {});
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_R", {});
    this->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    this->declare_parameter<bool>("publish.path_en", true);
    this->declare_parameter<bool>("publish.scan_publish_en", true);
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    this->declare_parameter<bool>("runtime_pos_log_enable", false);
    this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    this->declare_parameter<int>("pcd_save.interval", -1);

}

