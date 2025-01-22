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
#include <atomic>

#include <future>

// #include <omp.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <chrono>
#include <thread>
// #include <filesystem>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "utils/mypcl/pointType.h"
#include "relocalization/icp_3d.h"
#include "relocalization/scancontext.h"
#include "hj_interface/Pose.h"
#include <hj_interface/Ultra.h>
#include <hj_interface/LeftBack.h>
#include <hj_interface/LeftFront.h>
#include <hj_interface/LeftTof.h>
#include "function_factory.h"
#include "node_factory.h"
// #include "hjlog.h"
#include "log.h"
#include "hj_interface/RelocalizationResult.h"

#include "common_data/pose_data.h"
#include "common_data/left_tof_data.h"
#include "common_data/left_tof_data.h"

enum SonarIndex
{
    left_front = 0,
    left_back = 1,

};
// TODO(edan): delete using.
// using namespace aiper_relocalization_ns;
namespace aiper_relocalization_ns
{
    class Relocalization
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        Relocalization(const std::string &config_path);
        ~Relocalization();
        int buildMap();
        int timeStampSynchronization(double sonar_wave_timestamp);
        int timeStampSynchronizationRelocSource(double sonar_wave_timestamp);
        int Sonar2cloud(SonarIndex index, int index_sonar, int index_pose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud);
        int Sonar2cloud(SonarIndex index, int index_sonar, int index_pose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12);
        void buildMultiFrame();
        double calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2);
        void clusterPoses(double max_distance);
        void multiFrameCombined();
        void pubBuildingResult();
        void pubFailed();
        void buildRelocSource();
        bool run_icp(const std::pair<int, float> &detect_result);
        bool refineDetectionResult();
        bool reloc();
        std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segmentPointCloudIntoPtrs(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, size_t segment_size);
        double computeCurvature(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud);
        Eigen::Vector3d polynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud);
        double computeCurvature(double a, double b, double x);
        void CallbackSonar(const hj_interface::LeftFrontConstPtr &msg);
        void CallbackSonarT1(const hj_interface::LeftTofConstPtr &msg);
        void CallbackOdom(const hj_interface::PoseConstPtr &msg);
        bool findFirstLapEndTime();
        void run();
        void saveMap();
        void loadMap();
        std::vector<std::string> getFileNamesInDirectory(const std::string &directory_path);
        bool distanceToReloc();
        void mypcl2PointCloud2(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl);
        void reset();
        void start(const uint8_t &task_id);
        void clearData();
        void stop();
        void InitParams(const std::string &config_dir);
        bool isRelocSuccess();
        void pubResult(double& dis);
        void loadOptiResults();
        void getNonZeroIndices(Eigen::MatrixXd& mat, std::vector<std::pair<int, int>>& nonZeroIndices, std::vector<double>& nonValue);
        void loadDesc();
        void judgePointNum();
        void copy_directory(const boost::filesystem::path& source, const boost::filesystem::path& destination);
        std::string getCurrentTimeString();
        bool isRelocThreadStopped();
        bool isReceivedStopSignal();
        void DeterminePointAMapEdge(Eigen::Matrix4d& result,double& dis);
        template <typename Scalar>
        inline Eigen::Matrix<Scalar, 3, 1> R2ypr(const Eigen::Matrix<Scalar, 3, 3> &R)
        {
            Eigen::Matrix<Scalar, 3, 1> n = R.col(0);
            Eigen::Matrix<Scalar, 3, 1> o = R.col(1);
            Eigen::Matrix<Scalar, 3, 1> a = R.col(2);

            Eigen::Matrix<Scalar, 3, 1> ypr(3);
            double y = atan2(n(1), n(0));
            double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
            double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
            ypr(0) = y;
            ypr(1) = p;
            ypr(2) = r;

            return ypr;
        }

        typedef std::shared_ptr<Relocalization> Ptr;

    protected:
        // save map
        // TODO: pose, sonar wave, change to struct? or common struct?

        std::vector<common_data::PoseData, Eigen::aligned_allocator<common_data::PoseData>> _poses;
        std::vector<common_data::LeftTofData, Eigen::aligned_allocator<common_data::LeftTofData>> _sonar_wave_datas;
        std::deque<std::pair<int, double>> _pose_timestamp;
        std::deque<std::pair<int, double>> _sonar_wave_timestamp;

        std::deque<common_data::PoseData, Eigen::aligned_allocator<common_data::PoseData>> _pose_reloc_source;
        std::deque<common_data::LeftTofData, Eigen::aligned_allocator<common_data::LeftTofData>> _sonar_wave_datas_reloc_source;
        std::deque<std::pair<int, double>> _pose_timestamp_reloc_source;
        std::deque<std::pair<int, double>> _sonar_wave_timestamp_reloc_source;

        std::deque<common_data::PoseData, Eigen::aligned_allocator<common_data::PoseData>> _pose_reloc_second;
        std::deque<common_data::LeftTofData, Eigen::aligned_allocator<common_data::LeftTofData>> _sonar_wave_datas_reloc_second;
        std::deque<std::pair<int, double>> _pose_timestamp_reloc_second;
        std::deque<std::pair<int, double>> _sonar_wave_timestamp_reloc_second;

        double _left_front_fov_rad{15};
        double _left_front_x{0.156}; // sonar1坐标系相对base_link坐标系的x外参
        double _left_front_y{0.148}; // sonar1坐标系相对base_link坐标系的y外参
        // double _left_front_x{0.136};  // sonar1坐标系相对base_link坐标系的x外参
        // double _left_front_y{0.128};  // sonar1坐标系相对base_link坐标系的y外参
        double _left_front_yaw{1.57}; // sonar1坐标系相对base_link坐标系的yaw外参
        double _left_back_fov_rad{15};
        double _left_back_x{-0.161};  // sonar2坐标系相对base_link坐标系的x外参
        double _left_back_y{0.182};   // sonar2坐标系相对base_link坐标系的x外参
        double _left_back_yaw{1.701}; // sonar2坐标系相对base_link坐标系的yaw外参
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _left_front_cloud;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _left_back_cloud;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _cloud_all;

        bool _left_back_base{false};
        double _first_lap_end_time;

        std::vector<std::pair<int, int>> _p_sonarindedx_poseindex;
        std::vector<std::pair<int, int>> _p_sonarindedx_poseindex_reloc;
        // std::vector<std::pair<int, Eigen::Matrix4d>> _p_sonarindex_pose;
        // std::vector<std::pair<int, Eigen::Matrix4d>> _p_sonarindex_pose_reloc;
        // std::vector<std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>>> _clustered_poses;
        //std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose;
        // 定义一个对齐分配器，用于 std::pair 和 std::vector
        template <typename T>
        using aligned_allocator = Eigen::aligned_allocator<T>;

        // 定义对齐的 std::vector 和 std::pair 类型别名
        using PairIntMatrix4d = std::pair<int, Eigen::Matrix4d>;
        using AlignedVectorPairIntMatrix4d = std::vector<PairIntMatrix4d, aligned_allocator<PairIntMatrix4d>>;

        using PairIntDoubleMatrix4d = std::pair<int, std::pair<double, Eigen::Matrix4d>>;
        using AlignedVectorPairIntDoubleMatrix4d = std::vector<PairIntDoubleMatrix4d, aligned_allocator<PairIntDoubleMatrix4d>>;
        using AlignedClusteredPoses = std::vector<AlignedVectorPairIntDoubleMatrix4d, aligned_allocator<AlignedVectorPairIntDoubleMatrix4d>>;

        using PairCloudDoubleMatrix4d = std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>;
        using AlignedVectorPairCloudDoubleMatrix4d = std::vector<PairCloudDoubleMatrix4d, aligned_allocator<PairCloudDoubleMatrix4d>>;
        AlignedVectorPairIntMatrix4d _p_sonarindex_pose;
        AlignedVectorPairIntMatrix4d _p_sonarindex_pose_reloc;
        AlignedClusteredPoses _clustered_poses;
        AlignedVectorPairCloudDoubleMatrix4d _p_cloud_pose;

        double _distance_threshold{0.02}; // dis 0.5 detatime 20 dis 3 detatime 40
        // SCManager _sc_manager;
        std::shared_ptr<SCManager> _sc_manager_ptr;
        int _combine_frame{250};
        std::size_t _source_start_index{0};
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _reloc_source;
        // ros
        hj_bf::HJSubscriber _left_sonar_front_sub;
        hj_bf::HJSubscriber _odom_sub;
        std::atomic<bool>  _has_scancontext {false};
        bool _has_opti_results = false;
        std::atomic<bool> _ok {false};
        std::mutex _mutex_odom;
        std::mutex _mutex_sonar;
        bool _first_lap_end = false;
        std::thread _run;
        std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> _key_frames;
        double _reloc_distance{0};
        double _reloc_distance_threshold{5};
        double _reloc_source_distance_threshold{7.5};
        double _curvature_diff_max{30};
        double _first_lap_end_time_diff{100};
        double _first_lap_end_dis{0.3};
        double _time_diff_thre{500};
        common_data::PoseData _last_pose;
        bool _first_pose = true;
        bool _is_enough = false;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _map;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _target_cloud;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _icp_result_cloud;

        hj_bf::HJPublisher _map_pub;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr _reloc_cloud;
        hj_bf::HJPublisher _reloc_cloud_pub;
        std::vector<Eigen::Matrix4d> _kposes;
        int _map_size{0};
        double _icp_score_threshold{0.1};
        Eigen::Matrix4d _reloc_result;
        Eigen::Matrix4d _entry_position_result;
        bool _get_source_end_pose = false;
        Eigen::Matrix4d _source_end_pose;
        bool _get_start_pose = false;
        Eigen::Matrix4d _start_pose;
        std::atomic<bool> _pub_result{false};
        bool is_reloc_success_ = false;
        bool is_reloc_failed_ = false;
        bool _use_sim3{true};
        int _num_point_failed{0};
        bool _use_desc{true};
        std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> _sc_desc;
        sensor_msgs::PointCloud2 _cloud_pc2_map;
        std::atomic<bool> _start {true};
        std::atomic<bool> _is_received_stop_signal {false};
        std::atomic<bool> _is_reloc_thread_stopped {false};
        bool _use_mag_dire{true};
        double _dire_deg_thre{10};
        std::vector<std::vector<double>> _nonZeroIndices;
        std::vector<std::vector<double>> _nonValue;
        std::atomic<bool> _clear_data{false};
        //std::vector<Eigen::MatrixXd> _sc_desc;
        mypcl::PointXYZI _map_center;
        bool _dynamic_icp_score{true};


        // hjlog
        // void *_log = nullptr;
        std::string _log_path;

        // pub result
        uint8_t _task_id;
    };
} // namespace aiper_relocalization_ns

#endif
