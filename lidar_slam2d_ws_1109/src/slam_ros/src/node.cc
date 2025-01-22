#include "node.h"
#include "../../hj_slam_2d/util/point_cloud.h"
#include "../../hj_slam_2d/util/time.h"
#include "data_tool.h"

#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Core>

HJ_Slam2d_node::HJ_Slam2d_node(/* args */) : trajectory_builder(std::make_unique<hjSlam_2d::mapping::TrajectoryBuilder2D>())
{
    //trajectory_builder = std::make_unique<hjSlam_2d::mapping::TrajectoryBuilder2D>();
    trajectory_builder->setLocal_slam_result_callback(::std::bind(&HJ_Slam2d_node::OnLocalSlamResult, this,
                    ::std::placeholders::_1, ::std::placeholders::_2,
                    ::std::placeholders::_3, ::std::placeholders::_4));

    imu_sampler.setRatio(1.0);

    laserScan_sub = nh.subscribe("/horizontal_laser_2d_single_echo", 10, &HJ_Slam2d_node::laserMsgCallback, this);
    // odom_sub = nh.subscribe("odom", 10, &HJ_Slam2d_node::odomMsgCallback, this);
    imu_sub = nh.subscribe("/imu_temp", 10, &HJ_Slam2d_node::imuMsgCallback, this);
    hj_scan_sub = nh.subscribe("/hj_scan", 10, &HJ_Slam2d_node::startMapping, this); 
    posePublisher = nh.advertise<nav_msgs::Odometry>("hj_odom", 10);
    hj_laserScan_pub = nh.advertise<hj_slam::Lds>("hj_scan", 10);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("hj_map", 1, true);
    mapping_points_pub = nh.advertise<sensor_msgs::PointCloud2>("mapping_points", 1, true); //用于建图
    mappoints_to_matched = nh.advertise<sensor_msgs::PointCloud2>("mapping_points_to_matched", 1, true);  //targetCloud
    obstaclePoint_Pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle_points_to_matched", 1, true); //障碍物点
    path_pub = nh.advertise<nav_msgs::Path>("/laser/path", 1);

    // 初始化地图
    int length = ((xmax_ - xmin_) / delta_) * ((ymax_ - ymin_) / delta_);
    map_.map.data.resize(length);
    map_.map.info.resolution = delta_;
    map_.map.info.width = (xmax_ - xmin_) / delta_;
    map_.map.info.height = (ymax_ - ymin_) / delta_;
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;

    HJ_2dLaserMapping::Point center;
    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;
    matcher_map_ = new HJ_2dLaserMapping::ScanMatcherMap(center, xmin_, ymin_, xmax_, ymax_, delta_);

}

HJ_Slam2d_node::~HJ_Slam2d_node()
{
    
}

void HJ_Slam2d_node::laserMsgCallback(const sensor_msgs::LaserScanConstPtr &scan)
{

    if (!imu_sampler.Pulse())
    {
        return;
    }
    hjSlam_2d::PointCloudWithIntensities point_cloud;
    point_cloud= slam2dTool_ros::ToPointCloudWithIntensities(*scan);
    trajectory_builder->addRangeData(point_cloud);

} 

void HJ_Slam2d_node::odomMsgCallback(const nav_msgs::OdometryConstPtr &odom)
{
    auto odometry_data_ptr = slam2dTool_ros::ToOdometryData(odom);
    //trajectory_builder->AddOdometryData(*odometry_data_ptr);
}

void HJ_Slam2d_node::imuMsgCallback(const sensor_msgs::ImuConstPtr &imu)
{
    if (!imu_sampler.Pulse()) {
        return;
    }
    auto imu_data_ptr = slam2dTool_ros::ToImuData(imu);
    trajectory_builder->addImuData(*imu_data_ptr);
}

void HJ_Slam2d_node::startMapping(const hj_slam::Lds &msg)
{
    //=======================test  publish points=========================
    sensor_msgs::PointCloud2 point_cloud;

    hjSlam_2d::Lds mappint_points;
    mappint_points.ldsPose.pose.x = msg.ldsPose.pose.x;
    mappint_points.ldsPose.pose.y = msg.ldsPose.pose.y;
    mappint_points.ldsPose.pose.theta = msg.ldsPose.pose.theta;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < msg.points.size(); ++i)
    {
        pcl::PointXYZ p(msg.points[i].x, msg.points[i].y, 0);
        cloud->push_back(p);

        hjSlam_2d::LdsPoint pt;
        pt.x = msg.points[i].x;
        pt.y = msg.points[i].y;
        mappint_points.points.emplace_back(pt);
    }

    pcl::toROSMsg(*cloud, point_cloud);
    point_cloud.header.frame_id = "map";
    mapping_points_pub.publish(point_cloud);
    //================================================================
    begin_time = clock();
    matcher_.setlaserMaxRange(maxRange_);
    matcher_.setusableRange(maxUrange_);
    matcher_.registerScan2(*matcher_map_, mappint_points);

    // 根据smap地图中存储的栅格数据，修改map_.map.data[]的数据,这里为一维数组
    HJ_2dLaserMapping::OrientedPoint lp(msg.ldsPose.pose.x, msg.ldsPose.pose.y, msg.ldsPose.pose.theta);
    // 激光雷达在栅格地图上的栅格坐标p0
    HJ_2dLaserMapping::IntPoint p0 = matcher_map_->world2map(lp);
    int difference_pixels = update_window_size_ / delta_;
    int x_min = p0.x - difference_pixels < 0 ? 0 : (p0.x - difference_pixels);
    int y_min = p0.y - difference_pixels < 0 ? 0 : (p0.y - difference_pixels);
    int x_max = p0.x + difference_pixels > map_.map.info.width ? map_.map.info.width : (p0.x + difference_pixels);
    int y_max = p0.y + difference_pixels > map_.map.info.height ? map_.map.info.height : (p0.y + difference_pixels);

    //用于发布地图障碍物点
    hjSlam_2d::PointCloud map_obstacle_points;
    if (!got_map_)
    {
        for (int x = 0; x < matcher_map_->getMapSizeX(); x++)
        {
            for (int y = 0; y < matcher_map_->getMapSizeY(); y++)
            {
                // 从smap中得到栅格点p(x, y)被占用的概率
                HJ_2dLaserMapping::IntPoint p(x, y);
                double occ = matcher_map_->cell(p); // -1、0-1，到达一定的occ_thresh_认为被占用
                assert(occ <= 1.0);
                // 未知
                if (occ < 0)
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_UNKNOWN;
                // 占用
                else if (occ > occ_thresh_)
                {
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_OCC;
                }
                // 空闲
                else
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_FREE;
            }
        }
    }
    else
    {
        // 更新tof地图
        for (int x = x_min; x < x_max; x++)
        {
            for (int y = y_min; y < y_max; y++)
            {
                // 从smap中得到栅格点p(x, y)被占用的概率
                // -1、0-1，到达一定的occ_thresh_认为被占用
                HJ_2dLaserMapping::IntPoint p(x, y);
                double occ = matcher_map_->cell(p);
                assert(occ <= 1.0);

                // 未知
                if (occ < 0)
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_UNKNOWN;
                // 占用
                else if (occ > occ_thresh_)
                {
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_OCC;
                    float obstacle_mapX = x * 0.05 + xmin_ + 0.025;
                    float obstacle_mapY = y * 0.05 + ymin_ + 0.025;
                    hjSlam_2d::RangefinderPoint pt{Eigen::Vector3f(obstacle_mapX, obstacle_mapY, 0)};
                    map_obstacle_points.push_back(pt);
                }
                // 空闲
                else
                    map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = GMAPPING_FREE;
            }

        }
    }
    end_time = clock();
    std::cout << "============插入地图:==========" << double(end_time - begin_time) / 1000 << "ms" << std::endl << std::endl;

//================发布障碍物点云==========================
    if (map_obstacle_points.size() > 0)
    {
        sensor_msgs::PointCloud2 point_cloud2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < map_obstacle_points.size(); ++i)
        {
            pcl::PointXYZ p(map_obstacle_points[i].position.x(), map_obstacle_points[i].position.y(), 0);
            cloud->push_back(p);
        }
        pcl::toROSMsg(*cloud, point_cloud2);
        point_cloud2.header.frame_id = "map";
        obstaclePoint_Pub.publish(point_cloud2);
        trajectory_builder->setMapPointToMatched(map_obstacle_points);
    }

    got_map_ = true;
    // 把计算出来的地图发布出去
    map_.map.header.stamp = ros::Time::now();
    map_.map.header.frame_id = "map";

    // 发布map和map_metadata
    map_pub.publish(map_.map);

}

void HJ_Slam2d_node::OnLocalSlamResult(
    int trajectory_id,  hjSlam_2d::common::Time time,
    hjSlam_2d::transform::Rigid3d local_pose,
    ::hjSlam_2d::RangeData range_data_in_local)
{

    //*****************************发布里程计*************************
    nav_msgs::Odometry odom;
    odom.header.stamp = slam2dTool_ros::ToRos(time);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link"; // "base_footprint";
    odom.pose.pose.position.x = local_pose.translation().x();
    odom.pose.pose.position.y = local_pose.translation().y();
    odom.pose.pose.position.z = local_pose.translation().z();
    odom.pose.pose.orientation.w = local_pose.rotation().w();
    odom.pose.pose.orientation.x = local_pose.rotation().x();
    odom.pose.pose.orientation.y = local_pose.rotation().y();
    odom.pose.pose.orientation.z = local_pose.rotation().z();
    posePublisher.publish(odom);
    //*****************************发布tf******************************
    geometry_msgs::TransformStamped stamped_transform;
    stamped_transform.header.stamp = odom.header.stamp;
    stamped_transform.header.frame_id = "map";
    stamped_transform.child_frame_id = "base_link"; // "base_footprint";
    stamped_transform.transform.translation.x = odom.pose.pose.position.x;
    stamped_transform.transform.translation.y = odom.pose.pose.position.y;
    stamped_transform.transform.translation.z = odom.pose.pose.position.z;
    stamped_transform.transform.rotation.w =  odom.pose.pose.orientation.w;
    stamped_transform.transform.rotation.x = odom.pose.pose.orientation.x;
    stamped_transform.transform.rotation.y = odom.pose.pose.orientation.y;
    stamped_transform.transform.rotation.z = odom.pose.pose.orientation.z;
    tf_broadcaster_.sendTransform(stamped_transform);

    //***************************发布轨迹******************************
    static nav_msgs::Path imuPath;
    imuPath.header.stamp = odom.header.stamp;
    imuPath.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = odom.header.stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = odom.pose.pose;
    imuPath.poses.push_back(pose_stamped);
    path_pub.publish(imuPath);
    //**************************发布建图数据*************************
    hj_slam::Lds hj_scan;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);

    double roll, pitch, yaw;                      // 定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 进行转换

    hj_scan.ldsPose.pose.x = odom.pose.pose.position.x;
    hj_scan.ldsPose.pose.y = odom.pose.pose.position.y;
    hj_scan.ldsPose.pose.theta = yaw;

    for (int i = 0; i < range_data_in_local.returns.size(); i++)
    {
        hj_slam::LdsPoint p;
        p.x = range_data_in_local.returns[i].position.x();
        p.y = range_data_in_local.returns[i].position.y();
        hj_scan.points.emplace_back(p);
    }

    for (int i = 0; i < range_data_in_local.misses.size(); i++)
    {
        hj_slam::LdsPoint p;
        p.x = range_data_in_local.misses[i].position.x();
        p.y = range_data_in_local.misses[i].position.y();
        hj_scan.points.emplace_back(p);
    }

    hj_laserScan_pub.publish(hj_scan);

    sensor_msgs::PointCloud2 point_cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < range_data_in_local.points_to_matched.size(); ++i)
    {
        pcl::PointXYZ p(range_data_in_local.points_to_matched[i].position.x(), range_data_in_local.points_to_matched[i].position.y(), 0);
        cloud->push_back(p);
    }
    pcl::toROSMsg(*cloud, point_cloud2);
    point_cloud2.header.frame_id = "map";
    mappoints_to_matched.publish(point_cloud2);
}
