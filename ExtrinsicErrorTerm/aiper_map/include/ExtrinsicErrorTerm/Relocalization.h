#ifndef RELOCALIZATION_H
#define RELOCALIZATION_H
#define BOOST_TYPEOF_EMULATION
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <utility>
#include <memory>
#include <fstream>

#include <omp.h>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "ExtrinsicErrorTerm/pointType.h"
#include "ExtrinsicErrorTerm/icp_3d.h"
#include "ExtrinsicErrorTerm/Scancontext.h"
#include "hj_interface/Pose.h"
#include <hj_interface/Ultra.h>
#include "function_factory.h"
#include "node_factory.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <chrono>
#include <thread>
#include <filesystem>
enum SonarIndex
{
    left_front = 0,
    left_back = 1,

};

class Relocalization
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Relocalization();
    ~Relocalization();
    int buildMap();
    int timeStampSynchronization(double sonarWaveTimeStamp);
    int timeStampSynchronizationRelocSource(double sonarWaveTimeStamp);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12);
    void buildMultiFrame();
    double calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
    void clusterPoses(double maxDistance);
    void multiFrameCombined();
    void buildRelocSource();
    bool reloc();
    std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segmentPointCloudIntoPtrs(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud, size_t segmentSize);
    double computeCurvature(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud);
    Eigen::Vector3d polynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud);
    double computeCurvature(double a, double b, double x);
    Eigen::Vector3d ransacPolynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud, int iterations, double threshold);
    void CallbackSonar(const hj_interface::UltraConstPtr &msg);
    void CallbackOdom(const hj_interface::PoseConstPtr &msg);
    bool findFirstLapEndTime();
    void run();
    void saveMap();
    void loadMap();
    std::vector<std::string> getFileNamesInDirectory(const std::string& directoryPath);
    bool distanceToReloc();
    void mypcl2PointCloud2(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl);
    void mypcl2PointCloud2RGB(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl);
    void reset();

    typedef std::shared_ptr<Relocalization> Ptr;

protected:
    std::vector<std::vector<double>> _Poses;
    std::vector<std::vector<double>> _PosesRelocSource;
    std::vector<std::vector<double>> _OptimizedPoses;
    std::vector<int> _optimizedPoseIndex;
    std::vector<std::vector<double>> _SonarWaveDatas;
    std::vector<std::vector<double>> _SonarWaveDatasRelocSource;
    std::vector<std::vector<double>> _SonarWaveOptimizedDatas;
    std::deque<std::pair<int, double>> _PoseTimeStamp;
    std::deque<std::pair<int, double>> _PoseTimeStampRelocSource;
    std::deque<std::pair<int, double>> _SonarWaveTimeStamp;
    std::deque<std::pair<int, double>> _SonarWaveTimeStampRelocSource;
    double _leftFrontFovRad{15};
    double _leftFrontX{0.156}; // sonar1坐标系相对base_link坐标系的x外参
    double _leftFronty{0.148}; // sonar1坐标系相对base_link坐标系的y外参
    // double _leftFrontX{0.136};  // sonar1坐标系相对base_link坐标系的x外参
    // double _leftFronty{0.128};  // sonar1坐标系相对base_link坐标系的y外参
    double _leftFrontyaw{1.57}; // sonar1坐标系相对base_link坐标系的yaw外参
    double _leftBackFovRad{15};
    double _leftBackX{-0.161};  // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackY{0.182};   // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackYaw{1.701}; // sonar2坐标系相对base_link坐标系的yaw外参
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _leftFrontCloud;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _leftBackCloud;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _optimizedCloud;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _CloudAll;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _linecloud;
    
    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftBack;
    // pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftFront;
    bool _leftBackBase{false};
    bool _useAutoDiff{true};
    bool _usePlaneConstraints{false};
    bool _useLineConstraints{false};
    double _firstLapEndTime;
    std::vector<std::pair<int, int>> _p_sonarindedx_poseindex;
    std::vector<std::pair<int, int>> _p_sonarindedx_poseindex_reloc;
    std::vector<std::pair<int, Eigen::Matrix4d>> _p_sonarindex_pose;
    std::vector<std::pair<int, Eigen::Matrix4d>> _p_sonarindex_pose_reloc;
    std::vector<std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>>> _clustered_poses;
    std::vector<std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>>> _clustered_poses_ori;
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose;
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose_ori;
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _keyframes;
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _keyframes_show;

    double _distance_threshold{0.02}; // dis 0.5 detatime 20 dis 3 detatime 40
    SCManager _scManager;
    int _combineFrame{250};
    int _sourceStartIndex{0};
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _reloc_source;
    //ros
    hj_bf::HJSubscriber _left_sonar_front_sub;
    hj_bf::HJSubscriber _odom_sub;
    bool _hasScancontext = false;
    bool _ok = false;
    std::mutex _mutex_odom;
    std::mutex _mutex_sonar;
    bool _firstLapEnd = false;
    std::thread _run;
    std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> _keyFrames;
    double _relocDistance{0};
    std::vector<double>  _last_pose;
    bool _first_pose = true;
    bool _isEnough = false;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _map;
    hj_bf::HJPublisher  _map_pub;
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr _reloc_cloud;
    hj_bf::HJPublisher  _reloc_cloud_pub;
    std::vector<Eigen::Matrix4d> _kposes;
};

#endif
