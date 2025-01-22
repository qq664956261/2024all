#ifndef HJ_SLAM2D_NODE_H
#define HJ_SLAM2D_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "nav_msgs/GetMap.h"
#include "../../hj_slam_2d/core/matching/trajectory_builder_2d.h"
#include "../../hj_slam_2d/core/undistortion/pose_extrapolator.h"
#include "../../hj_slam_2d/core/mapping/visualization_map/scanmatcher/scanmatcher.h"
//#include "./visualization_map/scanmatcher/scanmatcher.h"
#include "tf2_ros/transform_broadcaster.h"
#include "hj_slam/Lds.h"
#include "../../hj_slam_2d/util/fixed_ratio_sampler.h"
#include <ctime>

#define   MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#define   GMAPPING_FREE         (0)
#define   GMAPPING_UNKNOWN      (-1)
#define   GMAPPING_OCC          (100)

class HJ_Slam2d_node
{
public:
    HJ_Slam2d_node(/* args */);
    ~HJ_Slam2d_node();

private:
    time_t begin_time, end_time;
    void laserMsgCallback(const sensor_msgs::LaserScanConstPtr &scan);
    void odomMsgCallback(const nav_msgs::OdometryConstPtr &odom);
    void imuMsgCallback(const sensor_msgs::ImuConstPtr &imu);
    void startMapping(const hj_slam::Lds &msg);

    void OnLocalSlamResult(int trajectory_id,
                           ::hjSlam_2d::common::Time time,
                           ::hjSlam_2d::transform::Rigid3d local_pose,
                           ::hjSlam_2d::RangeData range_data_in_local);

    std::unique_ptr<hjSlam_2d::mapping::TrajectoryBuilder2D> trajectory_builder;
    std::unique_ptr<hjSlam_2d::mapping::PoseExtrapolator> pose_extrapolator;

private:
    ros::NodeHandle nh;
    ros::Subscriber laserScan_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber hj_scan_sub;
    ros::Publisher posePublisher;
    ros::Publisher hj_laserScan_pub;
    ros::Publisher map_pub;
    ros::Publisher obstaclePoint_Pub;
    ros::Publisher path_pub;

    ros::Publisher mapping_points_pub;
    ros::Publisher mappoints_to_matched;

    HJ_2dLaserMapping::ScanMatcherMap *matcher_map_;
    HJ_2dLaserMapping::ScanMatcher matcher_;
    nav_msgs::GetMap::Response map_;    //用来发布map的实体对象
    hjSlam_2d::common::FixedRatioSampler imu_sampler;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
private:

    bool start_mapping_ = false;
    //pthread_mutex_t areaMutex;
    int m_getDectResult;
    bool tof_init = false;

    std::string scan_topic_ = "/hj_scan";
    std::string map_topic_ = "/explore_map";
    // 转 hj_scan 参数
    float angle_min = -M_PI; // -M_PI / 2.0;
    float angle_max = M_PI; // M_PI / 2.0;
    float angle_increment = M_PI / 360.0;
    float range_min = 0.05;
    float range_max = 35.0; //3.0;
    float min_height = -0.3;
    float max_height = 0.1;
    // 建图参数
    double map_update_interval_ = 0;
    bool got_map_ = false;
    double xmin_ = -200; //-30;
    double ymin_ = -200; //-30;
    double xmax_ = 200; //30;
    double ymax_ = 200; //30;
    double maxRange_ = 40.0;
    double maxUrange_ = 38.0;
    double delta_ = 0.05;
    double occ_thresh_ = 0.25;
    double m_linearThreshold_ = 0.0;
    double m_angularThreshold_ = 0.0;
    double update_window_size_ = 10;
};

#endif