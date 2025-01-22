#ifndef SONARBASE_H
#define SONARBASE_H

//#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/Range.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/point_cloud2_iterator.h>
//#include <nav_msgs/Odometry.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <visualization_msgs/MarkerArray.h>
//#include "utils.h"
//#include "function_factory.h"
//#include "node_factory.h"
//#include "ExtrinsicErrorTerm/pointType.h"
//
//
//class SonarBase
//{
//    public:
//        SonarBase();
//        // 可视化局部超声波数据
//        void displayLocalSonarPoint(const MapPointPtr& point);
//        void displayPose(const hj_mapping::OdomVec& odom_vec);
//        // display point cloud map
//        void displayLeftFrontSonarPoint(const MapPointPtrVec& point_stack);
//        void displayLeftBackSonarPoint(const MapPointPtrVec& point_stack);
//        void displayLocalPointCloud(const mypcl::MyPointCloud pc);
//        void displayLocalPointCloud2(const mypcl::MyPointCloud pc);
//        void toRosPontCloud(const std::vector<MapPointPtr> &src_point_cloud, sensor_msgs::PointCloud2 &cloud_msg);
//        void toRosPontCloud(const mypcl::MyPointCloud &point_cloud, sensor_msgs::PointCloud2 &cloud_msg);
//        void toMapPointPtrVec(const mypcl::MyPointCloud& pc, std::vector<MapPointPtr> &src_point_cloud);
//        // publish grid map
//        void publishSonarMap(nav_msgs::OccupancyGrid map);
//        inline SonarExteriorParam getSonarEPLf(){
//            return sonar_ep_lf_;
//        }
//        inline SonarExteriorParam getSonarEPLb(){
//            return sonar_ep_lb_;
//        }
//
//    private:
//        // 四元数转欧拉角
//        float toEulerAngle(const double x,const double y,const double z,const double w);
//
//        hj_bf::HJPublisher  cicle_pub_; // 圆弧的可视化发布者
//        hj_bf::HJPublisher  fitting_line_pub_;  // 拟合切线的可视化发布者
//        hj_bf::HJPublisher  tangent_line_pub_;  // 拟合切线的可视化发布者
//        hj_bf::HJPublisher  local_sonar_point_pub_; // 超声波点的可视化发布者
//        hj_bf::HJPublisher  sonar_point_pub_lf_;
//        hj_bf::HJPublisher  sonar_point_pub_lb_;
//        hj_bf::HJPublisher  sonar_map_pub_;
//        hj_bf::HJPublisher  pose_pub_;
//        hj_bf::HJPublisher  local_pc_pub2_;
//        hj_bf::HJPublisher  local_pc_pub_;
//        // 超声波到车体的外参
//        SonarExteriorParam  sonar_ep_lf_;
//        SonarExteriorParam  sonar_ep_lb_;
//
//        int  last_markers_count = 0;
//};
#endif
