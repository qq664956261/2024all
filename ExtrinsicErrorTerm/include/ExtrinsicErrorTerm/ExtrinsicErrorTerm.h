#ifndef EXTRINSICERRORTERM_H
#define EXTRINSICERRORTERM_H
#define BOOST_TYPEOF_EMULATION
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include <memory>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <omp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
#include "ExtrinsicErrorTerm/lidarFactor.h"
#include "ExtrinsicErrorTerm/aloam_analytic_factor.h"

enum SonarIndex
{
    left_front = 0,
    left_back = 1,

};
enum LaserFeatureType : int
{
    PCLHiPoints = 0,
    PCLOutlierPoints = 1,
    PCLCornerPoints = 2,
    PCLSurfPoints = 3,
    PCLOriPoints = 4,
};

struct LaserFeaturePoint
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    PCL_ADD_NORMAL4D;
    float IntensityVariance;
    float HeightVariance;
    int KeyFrameID;
    double Stamp;
    int FeatureType;
    double cov[16];


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LaserFeaturePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(float, IntensityVariance, IntensityVariance)(float, HeightVariance, HeightVariance)(int, KeyFrameID, KeyFrameID)(double, Stamp, Stamp)(int, FeatureType, FeatureType)
    (double, cov, cov)

)


class  ExtrinsicErrorTerm
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    ExtrinsicErrorTerm();
    ~ExtrinsicErrorTerm();
    int readPose(const std::string filepath);
    int readSonarWaveData(const std::string filepath);
    int buildMap();
    int timeStampSynchronization(double sonarWaveTimeStamp);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    int Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12);
    void align();
    void ceresAlign();
    typedef std::shared_ptr<ExtrinsicErrorTerm> Ptr;

protected:
    std::vector<std::vector<double>> _Poses;
    std::vector<std::vector<double>> _SonarWaveDatas;
    std::deque<std::pair<int,double>> _PoseTimeStamp;
    std::deque<std::pair<int,double>> _SonarWaveTimeStamp;
    double _leftFrontFovRad{15};          
    double _leftFrontX{0.156};  // sonar1坐标系相对base_link坐标系的x外参
    double _leftFronty{0.148};  // sonar1坐标系相对base_link坐标系的y外参
    // double _leftFrontX{0.136};  // sonar1坐标系相对base_link坐标系的x外参
    // double _leftFronty{0.128};  // sonar1坐标系相对base_link坐标系的y外参
    double _leftFrontyaw{1.57};// sonar1坐标系相对base_link坐标系的yaw外参
    double _leftBackFovRad{15};
    double _leftBackX{-0.161};  // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackY{0.182};   // sonar2坐标系相对base_link坐标系的x外参
    double _leftBackYaw{1.701}; // sonar2坐标系相对base_link坐标系的yaw外参
    pcl::PointCloud<pcl::PointXYZI>::Ptr _leftFrontCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _leftBackCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudAll;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _linecloud;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftBack;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr _kdtreeFromLeftFront;
    bool _leftBackBase {false};
    bool _useAutoDiff{true};
    bool _usePlaneConstraints{false};
    bool _useLineConstraints{false};
};

#endif
