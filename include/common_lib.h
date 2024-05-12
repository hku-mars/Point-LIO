#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <queue>

using namespace std;
using namespace Eigen;


typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_input,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((vect3, gravity))
);

MTK_BUILD_MANIFOLD(state_output,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, omg))
((vect3, acc))
((vect3, gravity))
((vect3, bg))
((vect3, ba))
);

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

MTK_BUILD_MANIFOLD(process_noise_input,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

MTK_BUILD_MANIFOLD(process_noise_output,
((vect3, vel))
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

extern esekfom::esekf<state_input, 24, input_ikfom> kf_input;
extern esekfom::esekf<state_output, 30, input_ikfom> kf_output;

#define PBWIDTH 30
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"

#define PI_M (3.14159265358)
// #define G_m_s2 (9.81)         // Gravaty const in GuangDong/China
#define DIM_STATE (24)      // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)      // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN  (6.0)
#define LIDAR_SP_LEN    (2)
#define INIT_COV   (0.0001)
#define NUM_MATCH_POINTS    (5)
#define MAX_MEAS_DIM        (10000)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define VEC_FROM_ARRAY_SIX(v)        v[0],v[1],v[2],v[3],v[4],v[5]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB     PointTypeRGB;
typedef pcl::PointCloud<PointType>    PointCloudXYZI;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

const M3D Eye3d(M3D::Identity());
const M3F Eye3f(M3F::Identity());
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_last_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_last_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
};

template <typename T>
T calc_dist(PointType p1, PointType p2){
    T d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

template <typename T>
T calc_dist(Eigen::Vector3d p1, PointType p2){
    T d = (p1(0) - p2.x) * (p1(0) - p2.x) + (p1(1) - p2.y) * (p1(1) - p2.y) + (p1(2) - p2.z) * (p1(2) - p2.z);
    return d;
}

template<typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr &point_cloud)
{
  int points_size = point_cloud->points.size();
  int j = 0;
  std::vector<int> time_seq;
  // time_seq.clear();
  time_seq.reserve(points_size);
  for(int i = 0; i < points_size - 1; i++)
  {
    j++;
    if (point_cloud->points[i+1].curvature > point_cloud->points[i].curvature)
    {
      time_seq.emplace_back(j);
      j = 0;
    }
  }
//   if (j == 0)
//   {
//     time_seq.emplace_back(1);
//   }
//   else
  {
    time_seq.emplace_back(j+1);
  }
  return time_seq;
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

inline double get_time_sec(const builtin_interfaces::msg::Time &time)
{
  return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp)
{
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}

#endif