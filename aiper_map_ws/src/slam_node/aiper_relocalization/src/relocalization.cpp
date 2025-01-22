#include <relocalization/relocalization.h>

namespace fs = boost::filesystem;
namespace aiper_relocalization_ns
{
    std::shared_ptr<aiper_relocalization_ns::KdTree> _map_kd_tree_ptr;
    Relocalization::Relocalization(const std::string &config_path)
    {
        std::string log_path = "/tmp/logging/reloc.log";
        // _log = hj_cst_log_add(log_path.c_str(), TRACE_LOG, 5 * 1024 * 1024, 3);
        HJ_INFO("Reloc > *******************relocalization start!!!*******************\n");
        HJ_INFO("Reloc > create log: %s !\n", log_path.c_str());
        HJ_INFO("Reloc > relocalization version: 1.0.0\n");
        HJ_ERROR("Reloc > *******************relocalization start!!!*******************\n");
        HJ_ERROR("Reloc > create log: %s !\n", log_path.c_str());
        HJ_ERROR("Reloc > relocalization version: 1.0.0\n");

        _ok = true;
        _reloc_result.setIdentity();
        _sc_manager_ptr = std::make_shared<SCManager>();
        _left_front_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _left_back_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _cloud_all.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _map.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _target_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _reloc_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _icp_result_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
// #ifdef X9
//         _left_sonar_front_sub = hj_bf::HJSubscribe("/x9/left_front", 1000, &Relocalization::CallbackSonar, this);
// #endif

// #ifdef T1_pro
//         _left_sonar_front_sub = hj_bf::HJSubscribe("/t1pro/left_tof", 1000, &Relocalization::CallbackSonarT1, this);
// #endif
        _left_sonar_front_sub = hj_bf::HJSubscribe("/x9/left_front", 1000, &Relocalization::CallbackSonar, this);
        _odom_sub = hj_bf::HJSubscribe("/fusion_result", 1000, &Relocalization::CallbackOdom, this);
        _map_pub = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/map", 1000);
        _reloc_cloud_pub = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/reloc_cloud", 1000);

        
        if (fs::exists("/userdata/hj/maps/map") && fs::exists("/userdata/hj/maps/point_cloud.bin"))
        {

            HJ_INFO("Reloc > detect the existence of the key frames... loading... \n");
            HJ_ERROR("Reloc > detect the existence of the key frames... loading... \n");
            _has_scancontext = true;
            loadMap();
        }
        else
        {
            if (fs::exists("./opti_result.txt"))
            {
                _has_opti_results = true;
                _has_scancontext = true;
                loadOptiResults();
            }
            HJ_INFO("Reloc > detect the key frame dose not exist... building key frames... \n");
            HJ_ERROR("Reloc > detect the key frame dose not exist... building key frames... \n");
        }
        InitParams(config_path);
        _run = std::thread(&Relocalization::run, this);
    }

    Relocalization::~Relocalization()
    {
        HJ_INFO("Reloc > ******************* destruct start!!!*******************\n");
        HJ_ERROR("Reloc > ******************* destruct start!!!*******************\n");
        _ok = false;
        if (_run.joinable())
        {
            _run.join();
        }
        // hj_cst_log_del(_log);
        HJ_INFO("Reloc > ******************* destruct success!!!*******************\n");
        HJ_ERROR("Reloc > ******************* destruct success!!!*******************\n");
    }

    void Relocalization::InitParams(const std::string &config_dir)
    {
        std::string config_path = config_dir + "/config/reloc_config.json";

        HJ_INFO("Reloc > open relocalization config file %s", config_path.c_str());
        HJ_ERROR("Reloc > open relocalization config file %s", config_path.c_str());
        // 打开配置文件
        std::ifstream config_file(config_path);
        if (!config_file.is_open())
        {
            HJ_ERROR("Unable to open relocalization config file %s", config_path.c_str());
            HJ_ERROR("Unable to open relocalization config file %s", config_path.c_str());
            return;
        }

        // 将配置文件解析为JSON
        nlohmann::json config;
        config_file >> config;

        // 关闭文件
        config_file.close();

        auto &relocalization = config["relocalization"];
        auto &ultra_param = relocalization["ultra_param"];
        _left_front_fov_rad = ultra_param["sonar_left_front_fov_rad"];
        _left_front_x = ultra_param["sonar_left_front_base_x"];
        _left_front_y = ultra_param["sonar_left_front_base_y"];
        _left_front_yaw = ultra_param["sonar_left_front_base_yaw"];
        _left_back_fov_rad = ultra_param["sonar_left_back_fov_rad"];
        _left_back_x = ultra_param["sonar_left_back_base_x"];
        _left_back_y = ultra_param["sonar_left_back_base_y"];
        _left_back_yaw = ultra_param["sonar_left_back_base_yaw"];
        _icp_score_threshold = relocalization["icp_score_threshold"];
        _distance_threshold = relocalization["distance_threshold"];
        _combine_frame = relocalization["combine_frame"];
        _use_sim3 = relocalization["use_sim3"];
        _reloc_distance_threshold = relocalization["reloc_distance_threshold"];
        _reloc_source_distance_threshold = relocalization["reloc_source_distance_threshold"];
        _curvature_diff_max = relocalization["curvature_diff_max"];
        _first_lap_end_dis = relocalization["first_lap_end_dis"];
        _first_lap_end_time_diff = relocalization["first_lap_end_time_diff"];
        _time_diff_thre = relocalization["time_diff_thre"];
        _use_mag_dire = relocalization["use_mag_dire"];
        _dire_deg_thre = relocalization["dire_deg_thre"];
        _dynamic_icp_score = relocalization["dynamic_icp_score"];
        _log_path = relocalization["log_path"];

        if(_dynamic_icp_score)
        {
            double map_point_num = _map->points.size();
            if(map_point_num > 3000 && map_point_num < 5000)
            {
                _icp_score_threshold = 0.05;
            }
            else if(map_point_num >= 5000 && map_point_num <= 8000)
            {
                _icp_score_threshold = 0.065;
            }
            else if(map_point_num > 8000)
            {
                _icp_score_threshold = 0.08;
            }
        }
    }

    int Relocalization::timeStampSynchronization(double sonar_wave_timestamp)
    {

        // double pose_stamp = _pose_timestamp.front().second * 1e-6;

        int index = _pose_timestamp.size() - 1;
        int index_l = 0;
        int index_r = 0;
        // bool picked = false;
        bool picked_l = false;
        bool picked_r = false;

        // 1. pick index
        for (int i = index; i >= 0; i--)
        {

            double timestamp = _pose_timestamp.at(i).second * 1e-6;
            if (timestamp - sonar_wave_timestamp > 0)
            {
                picked_r = true;
                index_r = i;
            }
            if (timestamp - sonar_wave_timestamp <= 0)
            {
                picked_l = true;
                index_l = i;
            }

            if (picked_r && picked_l)
            {
                if ((_pose_timestamp.at(index_r).second * 1e-6 - sonar_wave_timestamp) >= (sonar_wave_timestamp - _pose_timestamp.at(index_l).second * 1e-6))
                {

                    index = index_l;
                    break;
                }
                else
                {

                    index = index_r;
                    break;
                }
            }
        }
        if (picked_r && !picked_l)
            index = index_r;

        // // 排除数据
        // if (picked_r && picked_l)
        // {
        //     for (std::size_t i = 0; i <= index; i++)
        //     {
        //         _pose_timestamp.pop_front();
        //     }
        // }

        return _pose_timestamp.at(index).first;
    }

    int Relocalization::timeStampSynchronizationRelocSource(double sonar_wave_timestamp)
    {
        // double pose_stamp = _pose_timestamp_reloc_source.front().second * 1e-6;
        int index = _pose_timestamp_reloc_source.size() - 1;
        int index_l = 0;
        int index_r = 0;
        // bool picked = false;
        bool picked_l = false;
        bool picked_r = false;
        // 1. pick index
        for (int i = index; i >= 0; i--)
        {

            double timestamp = _pose_timestamp_reloc_source.at(i).second * 1e-6;

            if ((timestamp - sonar_wave_timestamp) > 0)
            {
                picked_r = true;
                index_r = i;
            }
            if ((timestamp - sonar_wave_timestamp) <= 0)
            {
                picked_l = true;
                index_l = i;
            }

            if (picked_r && picked_l)
            {
                if ((_pose_timestamp_reloc_source.at(index_r).second * 1e-6 - sonar_wave_timestamp) >= (sonar_wave_timestamp - _pose_timestamp_reloc_source.at(index_l).second * 1e-6))
                {

                    index = index_l;
                    break;
                }
                else
                {

                    index = index_r;
                    break;
                }
            }
        }
        if (picked_r && !picked_l)
        {
            index = index_r;
        }

        // // 排除数据
        // if (picked_r && picked_l)
        // {
        //     for (std::size_t i = 0; i <= index; i++)
        //     {
        //         _pose_timestamp.pop_front();
        //     }
        // }

        return _pose_timestamp_reloc_source.at(index).first;
    }

    void Relocalization::buildMultiFrame()
    {
        // HJ_INFO( "Frames > start build multi frames... \n");
        for (std::size_t i = 0; i < _sonar_wave_datas.size(); i++)
        {
            common_data::LeftTofData data = _sonar_wave_datas[i];
            double sonar_timestamp = double(data.time_) * 1e-6;
            int index = timeStampSynchronization(sonar_timestamp);
            Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
            Quatd cur_Q_ = Eigen::AngleAxisd(_poses[index].euler_[0], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(_poses[index].euler_[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(_poses[index].euler_[2], Eigen::Vector3d::UnitX());
            cur_Q_.normalize();
            T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
            T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_poses[index].p_[0], _poses[index].p_[1], _poses[index].p_[2]);
            _p_sonarindedx_poseindex.push_back(std::pair<int, int>(i, index));
            _p_sonarindex_pose.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));

            float sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
            double length;
            sonar_base_x = _left_front_x;
            sonar_base_y = _left_front_y;
            sonar_base_yaw = _left_front_yaw;
            length = double(data.front_distance_) / 1000.0;
            float theta_rad = 0.0 / 180.0 * M_PI;

            // sonar坐标系的数据
            float sonar_x, sonar_y;
            // sonar_z = 0.0;
            sonar_x = length * cos(theta_rad);
            sonar_y = length * sin(theta_rad);
            // sonar坐标系转base_link
            float base_x = sonar_base_x + (sonar_x * cos(sonar_base_yaw) - sonar_y * sin(sonar_base_yaw));
            float base_y = sonar_base_y + (sonar_x * sin(sonar_base_yaw) + sonar_y * cos(sonar_base_yaw));
            Eigen::Vector4d sonar_p(base_x, base_y, 0, 1);
            sonar_p = T_wc * sonar_p;
            mypcl::PointXYZI p;
            p.x = sonar_p[0];
            p.y = sonar_p[1];
            p.z = sonar_p[2];
            _map->points.push_back(p);
        }

        // HJ_INFO( "Frames > _p_sonarindedx_poseindex size: %zu", _p_sonarindedx_poseindex.size());
        //  bool first_pose = false;
        Eigen::Matrix4d first_T;
        clusterPoses(_distance_threshold);

        if (_clustered_poses.size() > 1)
        {
            double travel_distance = 0.0;
            AlignedVectorPairIntDoubleMatrix4d &last_group = _clustered_poses.back();

            // 计算最后一组中位姿依次行走的距离
            for (size_t i = 1; i < last_group.size(); ++i)
            {
                travel_distance += calculateDistance(last_group[i - 1].second.second, last_group[i].second.second);
            }

            // 如果累计行走的距离小于阈值，则合并至前一组
            // std::cout << "travel_distance:" << travel_distance << std::endl;
            if (travel_distance < _distance_threshold * 0.7)
            {
                // std::cout << "travel_distance < distance_threshold * 0.7" << std::endl;
                AlignedVectorPairIntDoubleMatrix4d &second_last_group = _clustered_poses[_clustered_poses.size() - 2];
                second_last_group.insert(second_last_group.end(), last_group.begin(), last_group.end());
                _clustered_poses.pop_back(); // 移除最后一组
            }
        }

        // std::cout << "multiFrameCombined" << std::endl;
        multiFrameCombined();
    }
    void Relocalization::multiFrameCombined()
    {
        HJ_INFO("Frames > _clustered_poses size: %zu", _clustered_poses.size());
        std::cout << "_clustered_poses.size():" << _clustered_poses.size() << std::endl;

        for (std::size_t i = 0; i < _clustered_poses.size(); i++)
        {
            AlignedVectorPairIntDoubleMatrix4d &cluster = _clustered_poses[i];
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
            // std::cout << "cluster.size():" << cluster.size() << std::endl;
            for (std::size_t j = 0; j < cluster.size(); j++)
            {
                int index = cluster[j].first;
                Eigen::Matrix4d T_wc = cluster[j].second.second;

                // float fov_rad = 0.0,
                // float sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
                float length = 0.0;
                // fov_rad = _left_front_fov_rad;
                // sonar_base_x = _left_front_x;
                // sonar_base_y = _left_front_y;
                // sonar_base_yaw = _left_front_yaw;
                length = double(_sonar_wave_datas[index].front_distance_) / 1000.0;
                // float half_fov = fov_rad / 2.0;
                float theta_rad = 0.0 / 180.0 * M_PI;

                // sonar坐标系的数据
                float sonar_x, sonar_y, sonar_z;
                sonar_z = 0.0;
                sonar_x = length * cos(theta_rad);
                sonar_y = length * sin(theta_rad);

                Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
                Eigen::Matrix4d T_base_front;
                T_base_front.setIdentity();
                Quatd q_base_front = Eigen::AngleAxisd(_left_front_yaw, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
                T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
                T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_left_front_x, _left_front_y, 0);

                Eigen::Matrix4d T_w_base = T_wc;
                Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
                Quatd q_w_front(T_w_front.block<3, 3>(0, 0));
                Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
                Eigen::Vector4d p_w_4;
                Eigen::Vector4d p_0; // 转到第一帧坐标系下
                p_w_4 = T_w_front * sonar_p;
                p_0 = _clustered_poses[i].back().second.second.inverse() * p_w_4;
                mypcl::PointXYZI p;
                p.x = p_0[0];
                p.y = p_0[1];
                p.z = p_0[2];
                cloud->points.push_back(p);
            }

            PairCloudDoubleMatrix4d p_cloud_time_pose;

            // p_cloud_time_pose.first = cloud;
            p_cloud_time_pose.first.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::copyPointCloud(*cloud, *p_cloud_time_pose.first);
            // p_cloud_time_pose.second.first = _clustered_poses[i][0].second.first;
            // p_cloud_time_pose.second.second = _clustered_poses[i][0].second.second;
            p_cloud_time_pose.second.first = _clustered_poses[i].back().second.first;
            p_cloud_time_pose.second.second = _clustered_poses[i].back().second.second;
            _p_cloud_pose.push_back(p_cloud_time_pose);
            _map_size += cloud->points.size();
        }
        // 新建 _p_cloud_pose_combined 来保存组合后的点云和位姿信息
        AlignedVectorPairCloudDoubleMatrix4d _p_cloud_pose_combined;
        // std::cout << "_p_cloud_pose.size():" << _p_cloud_pose.size() << std::endl;
        // std::cout << "_map_size.size():" << _map_size << std::endl;
        // 准备将 _p_cloud_pose 分成若干组，每组包含 _combine_frame 帧
        for (std::size_t i = 0; i < _p_cloud_pose.size(); i++)
        {
            int count = 0;
            auto combined_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

            // 定义组内时间戳和位姿的初始值
            double earliest_timestamp = _p_cloud_pose[i].second.first;
            Eigen::Matrix4d reference_pose = _p_cloud_pose[i].second.second;
            std::size_t j = i;
            Eigen::Matrix4d T1;
            double time_stamp;
            int point_size = 0;
            // while ((count < _combine_frame && j < _p_cloud_pose.size()) || (point_size < _map_size / 2 && j < _p_cloud_pose.size()))
            while ((point_size < _map_size / 2 && j < _p_cloud_pose.size()))
            {
                // 累加当前组内所有帧的点到 combined_cloud
                auto new_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
                mypcl::copyPointCloud(*(_p_cloud_pose[j].first), *new_cloud);
                Eigen::Matrix4d new_pose = _p_cloud_pose[j].second.second;
                time_stamp = _p_cloud_pose[j].second.first;
                // if (count == 0)
                // {
                T1 = new_pose;
                //}
                mypcl::transformPointCloud(*new_cloud, *new_cloud, new_pose.cast<float>());
                point_size += new_cloud->points.size();
                *combined_cloud += *(new_cloud);
                j++;

                if (_combine_frame > count)
                    count++;
            }
            // std::cout << "j:" << j << std::endl;
            // std::cout << "point_size:" << point_size << std::endl;

            mypcl::transformPointCloud(*combined_cloud, *combined_cloud, T1.inverse().cast<float>());
            // std::ofstream out("/userdata/hj/maps/map/kposes.csv", std::ios::app);
            // out << T1(0, 0) << " " << T1(0, 1) << " " << T1(0, 2) << " " << T1(0, 3) << " " << T1(1, 0) << " " << T1(1, 1) << " " << T1(1, 2) << " " << T1(1, 3) << " " << T1(2, 0) << " " << T1(2, 1) << " " << T1(2, 2)
            //     << " " << T1(2, 3) << " " << T1(3, 0) << " " << T1(3, 1) << " " << T1(3, 2) << " " << T1(3, 3) << std::endl;

            // 如果这是最后一组且它不足10帧，则与之前的点云合并
            // if (count < _combine_frame / 2 && !_p_cloud_pose_combined.empty())
            if (combined_cloud->points.size() < _map_size / 2)
            {
                //*(_p_cloud_pose_combined.back().first) += *combined_cloud;
                continue;
            }
            else
            {
                // 存储当前组的组合点云和第一帧的时间戳及位姿
                std::pair<double, Eigen::Matrix4d> time_pose = std::make_pair(time_stamp, T1);
                _p_cloud_pose_combined.push_back(std::make_pair(combined_cloud, time_pose));
            }

            if (combined_cloud->points.size() != 0)
            {
                // mypcl::savePLYFileBinary(std::to_string(i) + ".ply", *combined_cloud);
            }
        }

        _p_cloud_pose = _p_cloud_pose_combined;
    }

    void Relocalization::pubBuildingResult()
    {
        hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::RelocalizationResult>(
            "/relocalization_result_service");
        hj_interface::RelocalizationResult building_frames_result;
        building_frames_result.request.building_frames_result = 1;
        building_frames_result.request.relocalization_result = 0;
        client.call(building_frames_result);
        HJ_INFO("Frames > building frames success, pub result success...\n");
    }

    double Relocalization::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
    {
        return std::sqrt((pose1(0, 3) - pose2(0, 3)) * (pose1(0, 3) - pose2(0, 3)) +
                         (pose1(1, 3) - pose2(1, 3)) * (pose1(1, 3) - pose2(1, 3)));
    }

    void Relocalization::clusterPoses(double max_distance)
    {
        std::vector<bool> clustered(_p_sonarindex_pose.size(), false); // 标记位姿是否已经被分到某个簇中

        for (size_t i = 0; i < _p_sonarindex_pose.size(); ++i)
        {
            // 跳过已经聚类的位姿
            if (clustered[i])
                continue;

            AlignedVectorPairIntDoubleMatrix4d cluster;
            std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
            temp_pair.first = _p_sonarindex_pose[i].first;
            temp_pair.second = std::pair<double, Eigen::Matrix4d>(_poses[_p_sonarindedx_poseindex[i].second].time_, _p_sonarindex_pose[i].second);
            cluster.push_back(temp_pair);
            clustered[i] = true;
            int timestamp_index = _p_sonarindedx_poseindex[i].second;
            double timestamp = _poses[timestamp_index].time_ * 1e-6;

            // 对后续位姿进行检查，判断是否与当前位姿是同一簇的一部分
            for (size_t j = i + 1; j < _p_sonarindex_pose.size(); ++j)
            {
                if (!clustered[j] && calculateDistance(_p_sonarindex_pose[i].second, _p_sonarindex_pose[j].second) <= max_distance &&
                    std::abs(timestamp - _poses[_p_sonarindedx_poseindex[j].second].time_ * 1e-6) <= 40)
                {
                    temp_pair.first = _p_sonarindex_pose[j].first;
                    temp_pair.second = std::pair<double, Eigen::Matrix4d>(_poses[_p_sonarindedx_poseindex[j].second].time_, _p_sonarindex_pose[j].second);
                    cluster.push_back(temp_pair);
                    clustered[j] = true;
                }
            }
            _clustered_poses.push_back(cluster);
        }
        std::cout << "_clustered_poses.size():" << _clustered_poses.size() << std::endl;
    }

    void Relocalization::buildRelocSource()
    {
        for (std::size_t i = 0; i < _sonar_wave_datas_reloc_source.size(); i++)
        {
            common_data::LeftTofData tof_data = _sonar_wave_datas_reloc_source[i];
            double sonar_timestamp = double(tof_data.time_) * 1e-6;
            int index = timeStampSynchronizationRelocSource(sonar_timestamp);
            Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
            Quatd cur_Q_ = Eigen::AngleAxisd(_pose_reloc_source[index].euler_[0], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(_pose_reloc_source[index].euler_[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(_pose_reloc_source[index].euler_[2], Eigen::Vector3d::UnitX());
            cur_Q_.normalize();
            T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
            T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_pose_reloc_source[index].p_[0], _pose_reloc_source[index].p_[1], _pose_reloc_source[index].p_[2]);

            _p_sonarindedx_poseindex_reloc.push_back(std::pair<int, int>(i, index));
            _p_sonarindex_pose_reloc.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));
        }

        while (_source_start_index > _p_sonarindex_pose_reloc.size())
        {
            _source_start_index = _source_start_index / 2;
        }

        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> cluster;
        std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
        temp_pair.first = _p_sonarindex_pose_reloc[_source_start_index].first;
        temp_pair.second = std::pair<double, Eigen::Matrix4d>(_pose_reloc_source[_p_sonarindedx_poseindex_reloc[_source_start_index].second].time_, _p_sonarindex_pose_reloc[_source_start_index].second);
        cluster.push_back(temp_pair);
        // int timestamp_index = _p_sonarindedx_poseindex_reloc[_source_start_index].second;
        // double timestamp = _pose_reloc_source[timestamp_index][0] * 1e-6;

        double sum_distance = 0;
        for (size_t j = _source_start_index + 1; j < _p_sonarindex_pose_reloc.size(); ++j)
        {
            double distance = calculateDistance(_p_sonarindex_pose_reloc[j - 1].second, _p_sonarindex_pose_reloc[j].second);
            sum_distance += distance;
            // if (calculateDistance(_p_sonarindex_pose_reloc[_source_start_index].second, _p_sonarindex_pose_reloc[j].second) <= _distance_threshold * _combine_frame &&
            //     std::abs(timestamp - _pose_reloc_source[_p_sonarindedx_poseindex_reloc[j].second][0] * 1e-6) <= 40)
            // if (sum_distance <= _reloc_source_distance_threshold)
            // {
            temp_pair.first = _p_sonarindex_pose_reloc[j].first;
            temp_pair.second = std::pair<double, Eigen::Matrix4d>(_pose_reloc_source[_p_sonarindedx_poseindex_reloc[j].second].time_, _p_sonarindex_pose_reloc[j].second);
            cluster.push_back(temp_pair);
            // }
        }
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
        for (std::size_t j = 0; j < cluster.size(); j++)
        {
            int index = cluster[j].first;
            Eigen::Matrix4d T_wc = cluster[j].second.second;

            // float fov_rad = 0.0,
            // float sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
            double length;
            // fov_rad = _left_front_fov_rad;
            // sonar_base_x = _left_front_x;
            // sonar_base_y = _left_front_y;
            // sonar_base_yaw = _left_front_yaw;
            length = double(_sonar_wave_datas_reloc_source[index].front_distance_) / 1000.0;
            // float half_fov = fov_rad / 2.0;
            float theta_rad = 0.0 / 180.0 * M_PI;

            // sonar坐标系的数据
            float sonar_x, sonar_y, sonar_z;
            sonar_z = 0.0;
            sonar_x = length * cos(theta_rad);
            sonar_y = length * sin(theta_rad);
            Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
            Eigen::Matrix4d T_base_front;
            T_base_front.setIdentity();
            Quatd q_base_front = Eigen::AngleAxisd(_left_front_yaw, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
            T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_left_front_x, _left_front_y, 0);

            Eigen::Matrix4d T_w_base = T_wc;
            Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
            Quatd q_w_front(T_w_front.block<3, 3>(0, 0));
            Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
            Eigen::Vector4d p_w_4;
            Eigen::Vector4d p_0; // 转到最后一帧帧坐标系下
            p_w_4 = T_w_front * sonar_p;
            p_0 = cluster.back().second.second.inverse() * p_w_4;
            mypcl::PointXYZI p;
            p.x = p_0[0];
            p.y = p_0[1];
            p.z = p_0[2];
            cloud->points.push_back(p);
        }
        if ((!_get_source_end_pose))
        {
            _get_source_end_pose = true;
            _source_end_pose = cluster.back().second.second;
        }
        _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::copyPointCloud(*cloud, *_reloc_source);
        HJ_INFO("Reloc > reloc source size: %zu", _reloc_source->points.size());
        if (cloud->points.size() != 0)
        {
            std::string time = getCurrentTimeString();
            #ifdef RELOCALIZATION_DEBUG
            // mypcl::savePLYFileBinary("/userdata/hj/maps/map/" + time + "_source.ply", *cloud);
            mypcl::savePLYFileBinary("./" + time + "_source.ply", *cloud);
            #endif
        }
    }

    bool Relocalization::run_icp(const std::pair<int, float> &detect_result)
    {
        if (_use_desc)
        {
            mypcl::transformPointCloud(*_map, *_target_cloud, _kposes[detect_result.first].cast<float>().inverse());
        }
        else
        {
            mypcl::copyPointCloud(*(_key_frames[detect_result.first]), *_target_cloud);
        }

        Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rotation_vector(detect_result.second, Eigen::Vector3d(0, 0, 1));
        rotation_matrix = rotation_vector.toRotationMatrix();
        T.block<3, 3>(0, 0) = rotation_matrix;

        auto source_transformed = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::transformPointCloud(*_reloc_source, *source_transformed, T.inverse().cast<float>());
#ifdef RELOCALIZATION_DEBUG
        std::string time = getCurrentTimeString();
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/" + time + "_target_cloud.ply", *_target_cloud);
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/" + time + "_source_transformed.ply", *source_transformed);
        mypcl::savePLYFileBinary("./" + time + "_target_cloud.ply", *_target_cloud);
        mypcl::savePLYFileBinary("./" + time + "_source_transformed.ply", *source_transformed);
#endif
        aiper_relocalization_ns::Icp3d::Options options;
        options.max_iteration_ = 50;
        options.min_effective_pts_ = 20;
        options.eps_ = 1e-4;
        options.use_initial_translation_ = true;
        options.max_nn_distance_ = 0.5;
        aiper_relocalization_ns::Icp3d icp(options);
        mypcl::voxelGridFilter(*source_transformed, *source_transformed, 0.05);
        mypcl::voxelGridFilter(*_target_cloud, *_target_cloud, 0.05);
        icp.SetSource(source_transformed);
        icp.SetTarget(_target_cloud);

        SE3 pose;
        Eigen::Matrix4d T_init;
        T_init.setIdentity();
        Quatd q_init(T_init.block<3, 3>(0, 0));
        Eigen::Vector3d t_init(T_init.block<3, 1>(0, 3));
        pose = SE3(Quatd(q_init.w(), q_init.x(), q_init.y(), q_init.z()), Vec3d(t_init.x(), t_init.y(), t_init.z()));
        // bool success;
        // success =
        double s = 1;
        if (!_use_sim3)
        {
            icp.AlignP2P(pose);
        }
        else
        {
            icp.AlignP2LineSIM3(pose, s);
        }
        Eigen::Matrix4f icp_result = pose.matrix().cast<float>();
        icp_result = s * icp_result;
        // double score = icp.GetFitnessScore();
        double score = icp.GetFitnessScoreSIM3();
        
        HJ_ERROR("Reloc > icp score: %lf\n", score);
        HJ_ERROR("Reloc > sim3 icp s: %lf\n", s);

#ifdef RELOCALIZATION_DEBUG
        mypcl::transformPointCloud(*source_transformed, *_icp_result_cloud, icp_result.cast<float>());
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/" + time + "_icp_result_cloud.ply", *_icp_result_cloud);
        mypcl::savePLYFileBinary("./" + time + "_icp_result_cloud.ply", *_icp_result_cloud);
#endif
        if (score > _icp_score_threshold || s < 0.9 || s > 1.1)
        {
            return false;
        }
        else
        {
            _reloc_result = _kposes[detect_result.first] * icp_result.cast<double>() * T.inverse();
            return true;
        }
        return true;
    }

    bool Relocalization::refineDetectionResult()
    {
        auto icp_result_cloud_filter = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::voxelGridFilter(*_icp_result_cloud, *icp_result_cloud_filter, 0.05);
        auto target_cloud_filter = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::voxelGridFilter(*_target_cloud, *target_cloud_filter, 0.05);

        std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segments;
        segments = segmentPointCloudIntoPtrs(icp_result_cloud_filter, 10);
        HJ_INFO("Reloc > segments size: %zu\n", segments.size());

        auto icp_result_cloud_sample = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::voxelGridFilter(*icp_result_cloud_filter, *icp_result_cloud_sample, 2);

#ifdef RELOCALIZATION_DEBUG
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/icp_result_cloud_filter.ply", *icp_result_cloud_filter);
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/target_cloud_filter.ply", *target_cloud_filter);
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/icp_result_cloud_sample.ply", *icp_result_cloud_sample);
        mypcl::savePLYFileBinary("./icp_result_cloud_filter.ply", *icp_result_cloud_filter);
        mypcl::savePLYFileBinary("./target_cloud_filter.ply", *target_cloud_filter);
        mypcl::savePLYFileBinary("./icp_result_cloud_sample.ply", *icp_result_cloud_sample);
#endif

        std::shared_ptr<aiper_relocalization_ns::KdTree> kdtree_result = nullptr;
        kdtree_result = std::make_shared<aiper_relocalization_ns::KdTree>();
        kdtree_result->BuildTree(icp_result_cloud_filter);
        kdtree_result->SetEnableANN();
        std::shared_ptr<aiper_relocalization_ns::KdTree> kdtree_target = nullptr;
        kdtree_target = std::make_shared<aiper_relocalization_ns::KdTree>();
        kdtree_target->BuildTree(target_cloud_filter);
        kdtree_target->SetEnableANN();
        double diff_max = 0;

        for (std::size_t i = 0; i < icp_result_cloud_sample->points.size(); i++)
        {
            std::vector<int> nn_result;
            kdtree_result->GetClosestPoint(icp_result_cloud_sample->points[i], nn_result, 50); // 这里取最近邻
            std::vector<int> nn_target;
            kdtree_target->GetClosestPoint(icp_result_cloud_sample->points[i], nn_target, 50); // 这里取最近邻

            double average_result_x = 0;
            double average_result_y = 0;
            for (std::size_t j = 0; j < nn_result.size(); j++)
            {
                average_result_x += icp_result_cloud_filter->points[nn_result[j]].x;
                average_result_y += icp_result_cloud_filter->points[nn_result[j]].y;
            }
            average_result_x /= nn_result.size();
            average_result_y /= nn_result.size();
            double average_target_x = 0;
            double average_target_y = 0;
            for (std::size_t j = 0; j < nn_target.size(); j++)
            {
                average_target_x += target_cloud_filter->points[nn_target[j]].x;
                average_target_y += target_cloud_filter->points[nn_target[j]].y;
            }
            average_target_x /= nn_target.size();
            average_target_y /= nn_target.size();

            double diff_result_x = 0;
            double diff_result_y = 0;
            for (std::size_t j = 0; j < nn_result.size(); j++)
            {
                diff_result_x += icp_result_cloud_filter->points[nn_result[j]].x - icp_result_cloud_filter->points[nn_result[0]].x;
                diff_result_y += icp_result_cloud_filter->points[nn_result[j]].y - icp_result_cloud_filter->points[nn_result[0]].x;
            }
            double diff_target_x = 0;
            double diff_target_y = 0;
            for (std::size_t j = 0; j < nn_target.size(); j++)
            {
                diff_target_x += target_cloud_filter->points[nn_target[j]].x - icp_result_cloud_filter->points[nn_result[0]].x;
                diff_target_y += target_cloud_filter->points[nn_target[j]].y - icp_result_cloud_filter->points[nn_result[0]].x;
            }

            double diff_result = diff_result_x * diff_result_x + diff_result_y * diff_result_y;
            double diff_target = diff_target_x * diff_target_x + diff_target_y * diff_target_y;

            if (std::fabs(diff_result / diff_target) > diff_max)
                diff_max = std::fabs(diff_result / diff_target);

            auto nn_result_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
            for (std::size_t j = 0; j < nn_result.size(); j++)
            {
                nn_result_cloud->points.push_back(icp_result_cloud_filter->points[nn_result[j]]);
            }
            auto nn_target_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
            for (std::size_t j = 0; j < nn_target.size(); j++)
            {
                nn_target_cloud->points.push_back(target_cloud_filter->points[nn_target[j]]);
            }
        }
        HJ_INFO("Reloc > max curve diff: %lf\n", diff_max);
        if (std::fabs(diff_max) > _curvature_diff_max)
        {
            return false;
        }
        else
        {
            return true;
        }
        return true;
    }

    bool Relocalization::reloc()
    {
        auto start_build_source = std::chrono::high_resolution_clock::now();
        int sc_closest_frame_id = -1;
        float min_dist = 10000;
        std::pair<int, float> detect_result;
        buildRelocSource();
        auto end_build_source = std::chrono::high_resolution_clock::now();
        auto duration_build_source = std::chrono::duration_cast<std::chrono::milliseconds>(end_build_source - start_build_source);
        HJ_INFO("Reloc > buildRelocSource: %ld milliseconds to run\n", duration_build_source.count());
        auto start_make_key = std::chrono::high_resolution_clock::now();
        if (_use_desc)
        {
            double deta_yaw = 0;
            _sc_manager_ptr->makeAndSaveScancontextAndKeysSource(*_reloc_source);
            for (std::size_t i = 0; i < _nonZeroIndices.size(); i++)
            {
                Eigen::MatrixXd mat(30, 180);
                for (std::size_t j = 0; j < _nonZeroIndices[i].size() - 1; j = j + 2)
                {
                    if (_nonZeroIndices[i][j] > 30)
                        continue;
                    mat(_nonZeroIndices[i][j], _nonZeroIndices[i][j + 1]) = _nonValue[i][(j / 2)];
                }

                if (_use_mag_dire)
                {
                    Eigen::Matrix4d pose = _kposes[i];
                    Eigen::Matrix3d pose_r = pose.block<3, 3>(0, 0);
                    Eigen::Matrix3d r = _source_end_pose.block<3, 3>(0, 0);
                    // Eigen::Vector3d euler_r = R2ypr(pose_r);
                    // Eigen::Vector3d euler = R2ypr(r);
                    // deta_yaw = euler_r[0] - euler[0];
                    Eigen::Matrix3d deta_r =  pose_r.inverse() * r;
                    deta_yaw = R2ypr(deta_r)[0];
                    // HJ_INFO("Reloc > i: %ld\n", i);
                    // HJ_INFO("Reloc > deta_yaw: %lf\n", deta_yaw);
                }
                if (std::fabs(deta_yaw) > 0.017 * _dire_deg_thre)
                    continue;
                std::pair<float, float> result = _sc_manager_ptr->detectLoopClosureIDOneFrame(mat);
                if (result.first < 0.3)
                {
                    if (result.first < min_dist)
                    {
                        min_dist = result.first;
                        sc_closest_frame_id = i;
                        detect_result.first = sc_closest_frame_id;
                        detect_result.second = result.second;
                    }
                }
            }
        }
        else
        {
            for (std::size_t i = 0; i < _key_frames.size(); i++)
            {
                auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
                mypcl::copyPointCloud(*(_key_frames[i]), *cloud);
                _sc_manager_ptr->makeAndSaveScancontextAndKeys(*cloud);
            }
        }
        auto end_make_key = std::chrono::high_resolution_clock::now();
        auto duration_make_key = std::chrono::duration_cast<std::chrono::milliseconds>(end_make_key - start_make_key);
        HJ_INFO("Reloc > sc make key: %ld milliseconds to run\n", duration_make_key.count());

        if (!_use_desc)
        {
            _sc_manager_ptr->makeAndSaveScancontextAndKeys(*_reloc_source);

            auto start_find_id = std::chrono::high_resolution_clock::now();
            detect_result = _sc_manager_ptr->detectLoopClosureID(); // first: nn index, second: yaw diff
                                                                    // int sc_closest_frame_id = detect_result.first;
            auto end_find_id = std::chrono::high_resolution_clock::now();
            auto duration_find_id = std::chrono::duration_cast<std::chrono::milliseconds>(end_find_id - start_find_id);
            HJ_INFO("Reloc > sc find id: %ld milliseconds to run\n", duration_find_id.count());
        }
        HJ_INFO("Reloc > closest frame id: %d\n", sc_closest_frame_id);
        HJ_ERROR("Reloc > closest frame id: %d\n", sc_closest_frame_id);

        if (sc_closest_frame_id == -1)
        {
            judgePointNum();
            return false;
        }
        auto start_run_icp = std::chrono::high_resolution_clock::now();
        bool icp_success = run_icp(detect_result);
        auto end_run_icp = std::chrono::high_resolution_clock::now();
        auto duration_run_icp = std::chrono::duration_cast<std::chrono::milliseconds>(end_run_icp - start_run_icp);
        HJ_INFO("Reloc > sc run icp: %ld milliseconds to run\n", duration_run_icp.count());

        if (!icp_success)
        {
            HJ_INFO("Reloc > relocalization failed !!!\n");
            HJ_ERROR("Reloc > relocalization failed !!!\n");
            judgePointNum();
            return false;
        }

#ifdef RELOCALIZATION_DEBUG
        // ros显示
        auto final = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::transformPointCloud(*_icp_result_cloud, *final, _kposes[sc_closest_frame_id].cast<float>());
        sensor_msgs::PointCloud2 cloud_pc2;
        mypcl2PointCloud2(cloud_pc2, final);
        std::string time = getCurrentTimeString();
        // mypcl::savePLYFileBinary("/userdata/hj/maps/map/" + time + "_final.ply", *final);
        mypcl::savePLYFileBinary("./" + time + "_final.ply", *final);
        _reloc_cloud_pub.publish(cloud_pc2);
#endif

        // bool relocalization_success = refineDetectionResult();
        bool relocalization_success = true;
        if (relocalization_success)
        {
            HJ_INFO("Reloc > relocalization success !!!\n");
            HJ_ERROR("Reloc > relocalization success !!!\n");
            double dis = 0;
            pubResult(dis);
            if (dis > 0.4)
                return false;
            return true;
        }
        else
        {
            HJ_INFO("Reloc > relocalization failed !!!\n");
            HJ_ERROR("Reloc > relocalization failed !!!\n");
            judgePointNum();
            return false;
        }
        return true;
    }
    void Relocalization::pubResult(double& dis)
    {
        _mutex_odom.lock();
        _pub_result = true;
        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_pose_reloc_second.back().euler_[0], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(_pose_reloc_second.back().euler_[1], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(_pose_reloc_second.back().euler_[2], Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_pose_reloc_second.back().p_[0], _pose_reloc_second.back().p_[1], _pose_reloc_second.back().p_[2]);
        _mutex_odom.unlock();
        Eigen::Matrix4d T_first_cur = Eigen::Matrix4d::Identity();
        T_first_cur = _source_end_pose.inverse() * T_wc;


        _reloc_result = _reloc_result * T_first_cur;
        HJ_INFO("Reloc > reloc_result x: %lf\n", _reloc_result(0, 3));
        HJ_INFO("Reloc > reloc_result y: %lf\n", _reloc_result(1, 3));
        DeterminePointAMapEdge(_reloc_result, dis);
        if(dis > 0.4)
            return;
        HJ_INFO("Reloc > reloc_result DeterminePointAMapEdge x: %lf\n", _reloc_result(0, 3));
        HJ_INFO("Reloc > reloc_result DeterminePointAMapEdge y: %lf\n", _reloc_result(1, 3));
        
        //_entry_position_result = _reloc_result * _source_end_pose.inverse();
        _entry_position_result = _reloc_result * (T_wc.inverse() * _start_pose);
        HJ_INFO("Reloc > T_first_cur x: %lf\n", T_first_cur(0, 3));
        HJ_INFO("Reloc > T_first_cur y: %lf\n", T_first_cur(1, 3));
        HJ_INFO("Reloc > reloc x: %lf\n", _reloc_result(0, 3));
        HJ_INFO("Reloc > reloc y: %lf\n", _reloc_result(1, 3));

        Eigen::Matrix3d rotation = _reloc_result.block<3, 3>(0, 0);
        Eigen::Vector3d translation = _reloc_result.block<3, 1>(0, 3);
        Eigen::Quaterniond quaternion(rotation);

        Eigen::Matrix3d entry_rotation = _entry_position_result.block<3, 3>(0, 0);
        Eigen::Vector3d entry_translation = _entry_position_result.block<3, 1>(0, 3);
        Eigen::Quaterniond entry_quaternion(entry_rotation);
        HJ_INFO("Reloc > entry translation x: %lf\n", entry_translation.x());
        HJ_INFO("Reloc > entry translation y: %lf\n", entry_translation.y());

        hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::RelocalizationResult>(
            "/relocalization_result_service");
        hj_interface::RelocalizationResult reloc_result;
        reloc_result.request.pose.orientation.x = quaternion.x();
        reloc_result.request.pose.orientation.y = quaternion.y();
        reloc_result.request.pose.orientation.z = quaternion.z();
        reloc_result.request.pose.orientation.w = quaternion.w();
        reloc_result.request.pose.position.x = translation.x();
        reloc_result.request.pose.position.y = translation.y();
        reloc_result.request.pose.position.z = translation.z();
        reloc_result.request.entry_position_pose.orientation.x = entry_quaternion.x();
        reloc_result.request.entry_position_pose.orientation.y = entry_quaternion.y();
        reloc_result.request.entry_position_pose.orientation.z = entry_quaternion.z();
        reloc_result.request.entry_position_pose.orientation.w = entry_quaternion.w();
        reloc_result.request.entry_position_pose.position.x = entry_translation.x();
        reloc_result.request.entry_position_pose.position.y = entry_translation.y();
        reloc_result.request.entry_position_pose.position.z = entry_translation.z();
        reloc_result.request.relocalization_result = 1;
        reloc_result.request.building_frames_result = 0;
        reloc_result.request.task_id = _task_id;
        client.call(reloc_result);
    }

    std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> Relocalization::segmentPointCloudIntoPtrs(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, size_t segment_size)
    {
        std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segments;
        int sample_count = static_cast<int>(cloud->points.size() / segment_size);

        for (size_t i = 0; i < cloud->points.size(); i += sample_count)
        {
            size_t end = std::min(cloud->points.size(), i + sample_count);
            auto segment = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

            for (size_t j = i; j < end; ++j)
            {
                segment->points.push_back(cloud->points[j]);
            }

            segments.push_back(segment);
        }

        return segments;
    }

    // 基于给定点云拟合一个最佳圆，并计算该圆的曲率
    double Relocalization::computeCurvature(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
    {
        // 此示例省略了输入有效性检查
        // 创建矩阵和向量并填充
        Eigen::MatrixXd A(cloud->points.size(), 3);
        Eigen::VectorXd b(cloud->points.size());

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            A(i, 0) = cloud->points[i].x * 2;
            A(i, 1) = cloud->points[i].y * 2;
            A(i, 2) = -1.0;
            b(i) = -(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
        }

        // 求解圆的中心点 a, b 和半径的平方
        Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);
        double centerX = -solution(0);
        double centerY = -solution(1);
        double radiusSquared = centerX * centerX + centerY * centerY - solution(2);
        double radius = std::sqrt(radiusSquared);

        // 曲率的定义为 1/R
        double curvature = 1.0 / radius;
        return curvature;
    }

    // 为每个点及其邻域进行二次多项式拟合
    // 返回多项式系数：a, b, c
    Eigen::Vector3d Relocalization::polynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
    {
        Eigen::MatrixXd A(cloud->points.size(), 3);
        Eigen::VectorXd y(cloud->points.size()), x(3);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            A(i, 0) = cloud->points[i].x * cloud->points[i].x; // x^2
            A(i, 1) = cloud->points[i].x;                      // x
            A(i, 2) = 1.0;                                     // constant term
            y(i) = cloud->points[i].y;
        }

        // Solve Ax = y for x
        x = A.colPivHouseholderQr().solve(y);

        return x; // x contains the polynomial coefficients [a, b, c]
    }

    // 为给定的多项式系数a, b, c和点x计算曲率
    double Relocalization::computeCurvature(double a, double b, double x)
    {
        double firstDerivative = 2 * a * x + b;
        double secondDerivative = 2 * a;
        double curvature = std::fabs(secondDerivative) / std::pow((1 + firstDerivative * firstDerivative), 1.5);
        return curvature;
    }

    void Relocalization::CallbackOdom(const hj_interface::PoseConstPtr &msg)
    {
        std::unique_lock<std::mutex> lock(_mutex_odom);
        if (_start)
        {
            double x = msg->x;
            double y = msg->y;
            double z = msg->z;
            double yaw = msg->yaw;
            // double timestmap = msg->header.stamp.toNSec() * 1e-3;
            double timestmap = msg->timestamp.toNSec() * 1e-3;
            common_data::PoseData pose;
            pose.time_ = timestmap;
            pose.p_ = Eigen::Vector3d(x, y, z);
            pose.euler_ = Eigen::Vector3d(yaw, 0, 0);

            if (_has_scancontext)
            {

                if (!_is_enough)
                {
                    _pose_reloc_source.push_back(pose);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _pose_reloc_source.size() - 1;
                    tmp_pair.second = timestmap;
                    _pose_timestamp_reloc_source.push_back(tmp_pair);
                }
                else
                {
                    if (!_pub_result)
                    {
                        _pose_reloc_second.push_back(pose);
                        std::pair<int, double> tmp_pair;
                        tmp_pair.first = _pose_reloc_second.size() - 1;
                        tmp_pair.second = timestmap;
                        _pose_timestamp_reloc_second.push_back(tmp_pair);
                    }
                }
            }
            else
            {
                if (!_first_lap_end)
                {
                    _poses.push_back(pose);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _poses.size() - 1;
                    tmp_pair.second = timestmap;
                    _pose_timestamp.push_back(tmp_pair);
                }
            }
        }
    }

    void Relocalization::CallbackSonar(const hj_interface::LeftFrontConstPtr &msg)
    {
        std::unique_lock<std::mutex> lock(_mutex_sonar);
        if (_start)
        {
            double timestmap = msg->timestamp.toNSec() * 1e-3;
            double left_front_range = (double)msg->dist / 1000.0;
            double left_back_range = (double)msg->dist / 1000.0;
            if (left_front_range > 1)
                return;

            common_data::LeftTofData tof_data;
            tof_data.front_distance_ = msg->dist;
            tof_data.back_distance_ = msg->dist;
            tof_data.time_ = timestmap;
            _num_point_failed++;
            // HJ_INFO("num point failed: %d\n", _num_point_failed);
            if (_has_scancontext)
            {
                if (!_is_enough)
                {
                    _sonar_wave_datas_reloc_source.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas_reloc_source.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp_reloc_source.push_back(tmp_pair);
                }
                else
                {
                    // HJ_INFO("Reloc > data is enough, sonar wave datas reloc second size: %zu", _sonar_wave_datas_reloc_second.size());
                    _sonar_wave_datas_reloc_second.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas_reloc_second.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp_reloc_second.push_back(tmp_pair);
                }
            }
            else
            {
                if (!_first_lap_end)
                {
                    _sonar_wave_datas.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp.push_back(tmp_pair);
                }
            }
        }
    }
    void Relocalization::CallbackSonarT1(const hj_interface::LeftTofConstPtr &msg)
    {
        std::unique_lock<std::mutex> lock(_mutex_sonar);
        if (_start)
        {
            double timestmap = msg->timestamp.toNSec() * 1e-3;
            double left_front_range = (double)msg->dist_front / 1000.0;
            double left_back_range = (double)msg->dist_back / 1000.0;

            common_data::LeftTofData tof_data;
            tof_data.front_distance_ = msg->dist_front;
            tof_data.back_distance_ = msg->dist_back;
            tof_data.time_ = timestmap;
            if (left_front_range > 1)
                return;
            _num_point_failed++;
            // HJ_INFO("num point failed: %d\n", _num_point_failed);
            if (_has_scancontext)
            {
                if (!_is_enough)
                {
                    _sonar_wave_datas_reloc_source.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas_reloc_source.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp_reloc_source.push_back(tmp_pair);
                }
                else
                {
                    // HJ_INFO("Reloc > data is enough, sonar wave datas reloc second size: %zu", _sonar_wave_datas_reloc_second.size());
                    _sonar_wave_datas_reloc_second.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas_reloc_second.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp_reloc_second.push_back(tmp_pair);
                }
            }
            else
            {
                if (!_first_lap_end)
                {
                    _sonar_wave_datas.push_back(tof_data);
                    std::pair<int, double> tmp_pair;
                    tmp_pair.first = _sonar_wave_datas.size() - 1;
                    tmp_pair.second = timestmap;
                    _sonar_wave_timestamp.push_back(tmp_pair);
                }
            }
        }
    }

    bool Relocalization::isRelocSuccess()
    {
        return is_reloc_success_;
    }

    void Relocalization::run()
    {
        int ret = pthread_setname_np(pthread_self(), "aiper_relocalization");
        if (ret == 0) {
            HJ_INFO("success name aiper_relocalization %s", __FUNCTION__);
            HJ_ERROR("success name aiper_relocalization %s", __FUNCTION__);
        } else {
            HJ_INFO("failed name aiper_relocalization %s", __FUNCTION__);
            HJ_ERROR("failed name aiper_relocalization %s", __FUNCTION__);
        }
        while (_ok)
        {
            if (_start)
            {
                if (_has_scancontext)
                {
                    if (distanceToReloc())
                    {
                        if (!is_reloc_success_)
                        {
                            is_reloc_success_ = reloc();

                            if (is_reloc_success_)
                            {
                                //_ok = false;
                                // stop();
                                _is_reloc_thread_stopped.store(true);
                            }
                            else if (is_reloc_failed_)
                            {
                                //_ok = false;
                                // stop();
                                _is_reloc_thread_stopped.store(true);
                            }
                            else
                            {
                                reset();
                            }
                            if (_is_received_stop_signal.load()) {
                                _is_reloc_thread_stopped.store(true);
                            }
                        }
                    }
                }
                else
                {
                    // if (findFirstLapEndTime())
                    // {
                    //     buildMultiFrame();
                    //     saveMap();
                    //     pubBuildingResult();
                    //     //_ok = false;
                    //     // stop();
                    // }
                    pubFailed();
                     HJ_ERROR("reloc failed, no scancontext");
                }
                _map_pub.publish(_cloud_pc2_map);
            }
            
            if(_clear_data)
            {
                _clear_data = false;
                clearData();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    bool Relocalization::findFirstLapEndTime()
    {
        std::unique_lock<std::mutex> lock(_mutex_odom);
        if (_poses.size() < 2)
            return false;
        Eigen::Vector3d start_pose;
        Eigen::Vector3d end_pose;
        double start_time;
        double end_time;
        common_data::PoseData v_start_pose = _poses[0];
        start_pose = v_start_pose.p_;
        start_time = v_start_pose.time_;
        common_data::PoseData end_pose_vector = _poses.back();
        end_pose = end_pose_vector.p_;
        end_time = end_pose_vector.time_;
        double distance = std::sqrt((start_pose[0] - end_pose[0]) * (start_pose[0] - end_pose[0]) + (start_pose[1] - end_pose[1]) * (start_pose[1] - end_pose[1]));
        double time_diff = end_time * 1e-6 - start_time * 1e-6;

        HJ_INFO("Frames > start pose x: %lf, y: %lf, z: %lf\n", start_pose.x(), start_pose.y(), start_pose.z());

        HJ_INFO("Frames > end pose x: %lf, y: %lf, z: %lf\n", end_pose.x(), end_pose.y(), end_pose.z());

        HJ_INFO("Frames > start time: %lf, end time: %lf, time diff: %lf\n", start_time * 1e-6, end_time * 1e-6, time_diff);

        HJ_INFO("Frames > distance: %lf\n", distance);

        if (time_diff > _first_lap_end_time_diff && distance < _first_lap_end_dis)
        {
            _first_lap_end_time = end_time;
            std::cout << "Frames > end_time:" << _first_lap_end_time << std::endl;
            _first_lap_end = true;
            return true;
        }

        if (time_diff > _time_diff_thre)
        {
            HJ_INFO("time_diff > _time_diff_thre!!!!!!!!!!!!!!!!!");
            return true;
        }

        return false;
    }

    void Relocalization::saveMap()
    {
        // std::error_code ec;
        // std::filesystem::create_directories("/userdata/hj/maps/map", ec);
        fs::path dir("/userdata/hj/maps/map");
        boost::system::error_code ec; // 用于捕获错误信息
        fs::create_directories(dir, ec);
        std::ofstream out("/userdata/hj/maps/map/kposes.csv", std::ios::app);
        std::ofstream out_desc("/userdata/hj/maps/map/desc.csv", std::ios::app);
        for (std::size_t i = 0; i < _p_cloud_pose.size(); i++)
        {
            auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
            mypcl::copyPointCloud(*(_p_cloud_pose[i].first), *cloud);
            if (_use_desc)
            {
                auto sc_matrix = _sc_manager_ptr->makeScancontext(*cloud);
                std::vector<std::pair<int, int>> nonZeroIndices;
                std::vector<double> nonValue;
                getNonZeroIndices(sc_matrix, nonZeroIndices, nonValue);
                // std:;cout<<"_p_cloud_pose.size():"<<_p_cloud_pose.size()<<std::endl;
                // std::cout << "nonZeroIndices.size():" << nonZeroIndices.size() << std::endl;
                // std::cout << "nonValue.size():" << nonValue.size() << std::endl;
                for (std::size_t j = 0; j < nonZeroIndices.size(); j++)
                {
                    if (j == nonZeroIndices.size() - 1)
                    {
                        out_desc << nonZeroIndices[j].first << " " << nonZeroIndices[j].second << std::endl;
                    }
                    else
                    {
                        out_desc << nonZeroIndices[j].first << " " << nonZeroIndices[j].second << " ";
                    }
                }

                for (std::size_t j = 0; j < nonValue.size(); j++)
                {
                    if (j == nonValue.size() - 1)
                    {
                        out_desc << nonValue[j] << std::endl;
                    }
                    else
                    {
                        out_desc << nonValue[j] << " ";
                    }
                }
            }
            else
            {
                mypcl::savePLYFileBinary("/userdata/hj/maps/map/cloud_" + std::to_string(i) + ".ply", *cloud);
            }
            // std::cout << "/userdata/hj/maps/map/cloud_" + std::to_string(i) + ".ply" << std::endl;
            Eigen::Matrix4d T = _p_cloud_pose[i].second.second;
            out << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " " << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " " << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << " " << T(3, 0) << " " << T(3, 1) << " " << T(3, 2) << " " << T(3, 3);
            out << std::endl;
        }
        HJ_INFO("Frames > save map successfull!!!");
        // mypcl::transformPointCloud(*_map,*_map,_p_cloud_pose[0].second.second.cast<float>().inverse());

        mypcl::savePLYFileBinary("/userdata/hj/maps/map/map.ply", *_map);
    }

    // std::vector<std::string> Relocalization::getFileNamesInDirectory(const std::string &directory_path)
    // {
    //     std::vector<std::string> file_names;

    //     for (const auto &entry : std::filesystem::directory_iterator(directory_path))
    //     {
    //         if (std::filesystem::is_regular_file(entry))
    //         {
    //             file_names.push_back(entry.path().filename().string());
    //         }
    //     }
    //     return file_names;
    // }
    std::vector<std::string> Relocalization::getFileNamesInDirectory(const std::string &directory_path)
    {
        std::vector<std::string> file_names;

        fs::path dir_path(directory_path);
        if (fs::exists(dir_path) && fs::is_directory(dir_path)) // 确保目录存在
        {
            fs::directory_iterator end_iter; // 默认构造产生的迭代器作为结束迭代器
            for (fs::directory_iterator dir_iter(dir_path); dir_iter != end_iter; ++dir_iter)
            {
                if (fs::is_regular_file(dir_iter->status())) // 如果是普通文件
                {
                    // 添加文件名到列表
                    file_names.push_back(dir_iter->path().filename().string());
                }
            }
        }
        return file_names;
    }
    void Relocalization::loadMap()
    {
        HJ_INFO("Reloc > detect map exists, start load map !!!");
        HJ_ERROR("Reloc > detect map exists, start load map !!!");
        std::vector<std::string> file_names = getFileNamesInDirectory("/userdata/hj/maps/map");
        if (_use_desc)
        {
            loadDesc();
        }
        else
        {
            _key_frames.clear();
            for (std::size_t i = 0; i < file_names.size(); i++)
            {
                auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
                mypcl::loadPLYFile("/userdata/hj/maps/map/cloud_" + std::to_string(i) + ".ply", *cloud);
                // std::cout << "/userdata/hj/maps/map/cloud_" + std::to_string(i) + ".ply" << std::endl;
                _key_frames.push_back(cloud);
            }
        }
        _map.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::loadPLYFile("/userdata/hj/maps/map/map.ply", *_map);
        for(int i = 0; i < _map->points.size(); i++)
        {
            _map_center.x += _map->points[i].x;
            _map_center.y += _map->points[i].y;
        }
        _map_center.x /= _map->points.size();
        _map_center.y /= _map->points.size();

        _map_kd_tree_ptr = std::make_shared<aiper_relocalization_ns::KdTree>();
        _map_kd_tree_ptr->BuildTree(_map);
        _map_kd_tree_ptr->SetEnableANN();


        mypcl2PointCloud2(_cloud_pc2_map, _map);
        _map_pub.publish(_cloud_pc2_map);

        std::string pose_file_names = "/userdata/hj/maps/map/kposes.csv";
        std::ifstream infile(pose_file_names.c_str());
        // int count = 0;
        _kposes.clear();
        for (std::string line; std::getline(infile, line);)
        {
            std::stringstream ss(line);
            std::string str;
            std::vector<std::string> line_array;
            std::vector<double> data;
            while (getline(ss, str, ' '))
            {
                line_array.push_back(str);
            }
            for (std::size_t i = 0; i < line_array.size(); i++)
            {
                std::stringstream ss;
                double temp_data;
                ss << line_array[i];
                ss >> temp_data;
                data.push_back(temp_data);
            }
            Eigen::Matrix4d T;
            T << data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15];
            _kposes.push_back(T);
        }
        HJ_INFO("Reloc > detect map exists, load map successfull !!!");
        HJ_ERROR("Reloc > detect map exists, load map successfull !!!");
    }

    bool Relocalization::distanceToReloc()
    {
        // TODO: delete lock? zhen.liang
        std::unique_lock<std::mutex> lock(_mutex_odom);
        if (_pose_reloc_source.size() < 2)
            return false;

        if (!_get_start_pose)
        {
            common_data::PoseData pose = _pose_reloc_source.back();
            _start_pose.setIdentity();
            _start_pose.block<3, 1>(0, 3) = pose.p_;
            _start_pose.block<3, 3>(0, 0) = pose.q_.toRotationMatrix();
            _get_start_pose = true;
        }

        if (_first_pose)
        {
            _first_pose = false;
            _last_pose = _pose_reloc_source[0];
            return false;
        }

        Eigen::Vector3d start_pose;
        Eigen::Vector3d end_pose;

        start_pose = _last_pose.p_;
        // start_time = _last_pose[0];
        common_data::PoseData end_pose_vector = _pose_reloc_source.back();
        end_pose << end_pose_vector.p_;
        // end_time = end_pose_vector[0];
        double distance = std::sqrt((start_pose[0] - end_pose[0]) * (start_pose[0] - end_pose[0]) + (start_pose[1] - end_pose[1]) * (start_pose[1] - end_pose[1]));
        _reloc_distance += distance;
        HJ_INFO("Reloc > start pose x: %f, y: %f, z: %f\n", start_pose.x(), start_pose.y(), start_pose.z());
        HJ_INFO("Reloc > end pose x: %f, y: %f, z: %f\n", end_pose.x(), end_pose.y(), end_pose.z());
        HJ_INFO("Reloc > reloc distance: %f\n", _reloc_distance);
        HJ_INFO("Reloc > sonar size: %zu", _sonar_wave_datas_reloc_source.size());
        HJ_INFO("Reloc > map size: %zu", _map->points.size());
        int sonar_size = _sonar_wave_datas_reloc_source.size();
        int map_size = _map->points.size();

         if (_reloc_distance > _reloc_distance_threshold && sonar_size > map_size * 1 / 2)
        {
            // if(_pose_reloc_source.back()[0] * 1e-6 - _pose_reloc_source[0][0] * 1e-6 > 100)
            // {
            HJ_INFO("Reloc > reloc distance > reloc distance threshold and sonar size > map size * 0.5 !!!\n");
            HJ_INFO("Reloc > data is enough !!!\n");
            _is_enough = true;
            return true;
        }
        else
        {
            HJ_INFO("Reloc > data is not enough !!!\n");
            _last_pose = end_pose_vector;
            return false;
        }
    }

    void Relocalization::mypcl2PointCloud2(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl)
    {
        HJ_INFO("Reloc > map size: %zu\n", cloud_mypcl->points.size());
        // 设置头信息
        cloud_pc2.header.frame_id = "odom";
        cloud_pc2.header.stamp = ros::Time::now();

        // 设置点云字段
        sensor_msgs::PointCloud2Modifier modifier(cloud_pc2);
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(cloud_mypcl->points.size());

        // 直接使用迭代器填充点云数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_pc2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_pc2, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_pc2, "z");
        for (size_t i = 0; i < cloud_mypcl->points.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = static_cast<float>(cloud_mypcl->points[i].x);
            *iter_y = static_cast<float>(cloud_mypcl->points[i].y);
            *iter_z = static_cast<float>(cloud_mypcl->points[i].z);
        }
    }

    void Relocalization::reset()
    {
        HJ_INFO("Reloc > reloclization reset start!!!");
        HJ_ERROR("Reloc > reloclization reset start!!!");
        std::unique_lock<std::mutex> sonar_lock(_mutex_sonar);
        std::unique_lock<std::mutex> odom_lock(_mutex_odom);
        //_reloc_distance = 0;
        _first_pose = true;
        _is_enough = false;
        _pub_result = false;
        _reloc_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>());
        _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);

        _sc_manager_ptr = std::make_shared<SCManager>();
        _map_size = 0;
        _first_lap_end_time = 0;
        _get_source_end_pose = false;
        _p_sonarindedx_poseindex.clear();
        _p_sonarindedx_poseindex_reloc.clear();
        _p_sonarindex_pose.clear();
        _p_sonarindex_pose_reloc.clear();
        _clustered_poses.clear();
        _p_cloud_pose.clear();
        HJ_INFO("Reloc > pose reloc size ori: %zu", _pose_reloc_source.size());
        HJ_ERROR("Reloc > pose reloc size ori: %zu", _pose_reloc_source.size());
        _pose_reloc_source.insert(_pose_reloc_source.end(), _pose_reloc_second.begin(), _pose_reloc_second.end());
        HJ_INFO("Reloc > pose reloc size add: %zu", _pose_reloc_source.size());
        HJ_ERROR("Reloc > pose reloc size add: %zu", _pose_reloc_source.size());
        _sonar_wave_datas_reloc_source.insert(_sonar_wave_datas_reloc_source.end(), _sonar_wave_datas_reloc_second.begin(), _sonar_wave_datas_reloc_second.end());
        _pose_timestamp_reloc_source.insert(_pose_timestamp_reloc_source.end(), _pose_timestamp_reloc_second.begin(), _pose_timestamp_reloc_second.end());
        _sonar_wave_timestamp_reloc_source.insert(_sonar_wave_timestamp_reloc_source.end(), _sonar_wave_timestamp_reloc_second.begin(), _sonar_wave_timestamp_reloc_second.end());

        int map_size = _map->points.size();
        if ((_sonar_wave_datas_reloc_source.size()) > (map_size / 2 + 100))
        {
            HJ_INFO("Reloc > berore sonar wave datas reloc  size: %zu", _sonar_wave_datas_reloc_source.size());
            //_sonar_wave_datas_reloc_source.erase(_sonar_wave_datas_reloc_source.begin(), _sonar_wave_datas_reloc_source.begin() + (_sonar_wave_datas_reloc_source.size() - _map_size/2 - 100));
            while ((_sonar_wave_datas_reloc_source.size()) > (map_size / 2 + 100))
            {
                if(_sonar_wave_datas_reloc_source.size() > 0)
                {
                _sonar_wave_datas_reloc_source.pop_front();
                }
            }
            HJ_INFO("Reloc > sonar wave datas reloc second size: %zu", _sonar_wave_datas_reloc_second.size());
            HJ_INFO("Reloc > after sonar wave datas reloc  size: %zu", _sonar_wave_datas_reloc_source.size());
        }
        else
        {
            std::cout << "Error: Trying to erase more elements than the deque contains.\n";
        }
        if ((_sonar_wave_timestamp_reloc_source.size()) > (map_size / 2 + 100))
        {
            HJ_INFO("Reloc > before sonar wave timestamp reloc size: %zu", _sonar_wave_timestamp_reloc_source.size());
            HJ_INFO("Reloc > sonar wave timestamp reloc second size: %zu", _sonar_wave_timestamp_reloc_second.size());
            //_sonar_wave_timestamp_reloc_source.erase(_sonar_wave_timestamp_reloc_source.begin(), _sonar_wave_timestamp_reloc_source.begin() + (_sonar_wave_timestamp_reloc_source.size() - _map_size/2 - 100));
            while ((_sonar_wave_timestamp_reloc_source.size()) > (map_size / 2 + 100))
            {
           
                _sonar_wave_timestamp_reloc_source.pop_front();
                
            }
            HJ_INFO("Reloc > after sonar wave timestamp reloc size: %zu", _sonar_wave_timestamp_reloc_source.size());
        }
        else
        {
            std::cout << "Error: Trying to erase more elements than the deque contains.\n";
        }
        // if (_pose_reloc_second.size() <= _pose_reloc_source.size())
        // {
        //     HJ_INFO("Reloc > pose reloc second size: %zu", _pose_reloc_second.size());
        //     //_pose_reloc_source.erase(_pose_reloc_source.begin(), _pose_reloc_source.begin() + _pose_reloc_second.size());
        // }
        // else
        // {
        //     std::cout << "Error: Trying to erase more elements than the deque contains.\n";
        // }

        // if (_pose_timestamp_reloc_second.size() <= _pose_timestamp_reloc_source.size())
        // {
        //     HJ_INFO("Reloc > pose timestamp reloc second size: %zu", _pose_timestamp_reloc_second.size());
        //     //_pose_timestamp_reloc_source.erase(_pose_timestamp_reloc_source.begin(), _pose_timestamp_reloc_source.begin() + _pose_timestamp_reloc_second.size());
        // }
        // else
        // {
        //     std::cout << "Error: Trying to erase more elements than the deque contains.\n";
        // }
        if(_sonar_wave_timestamp_reloc_source.size() > 0 && _pose_timestamp_reloc_source.size() > 0)
        {
        while ((_sonar_wave_timestamp_reloc_source.front().second * 1e-6 - _pose_timestamp_reloc_source.front().second * 1e-6) > 1)
        {
            _pose_timestamp_reloc_source.pop_front();
            _pose_reloc_source.pop_front();
        }
        }
        for (int i = 0; i < _sonar_wave_timestamp_reloc_source.size(); i++)
        {
            _sonar_wave_timestamp_reloc_source[i].first = i;
        }
        for (int i = 0; i < _pose_timestamp_reloc_source.size(); i++)
        {
            _pose_timestamp_reloc_source[i].first = i;
        }
        _pose_reloc_second.clear();
        _sonar_wave_datas_reloc_second.clear();
        _pose_timestamp_reloc_second.clear();
        _sonar_wave_timestamp_reloc_second.clear();
        HJ_INFO("Reloc > reloclization reset successfull!!!");
        HJ_ERROR("Reloc > reloclization reset successfull!!!");
    }
    void Relocalization::start(const uint8_t &task_id)
    {
        HJ_INFO("Reloc > reloclization start!!!");
        HJ_ERROR("Reloc > reloclization start!!!");
        _start = true;
        _is_reloc_thread_stopped.store(false);
        _is_received_stop_signal.store(false);
        _task_id = task_id;
        if (!_has_scancontext)
        {
            if (fs::exists("/userdata/hj/maps/map") && fs::exists("/userdata/hj/maps/point_cloud.bin"))
            {
                _has_scancontext = true;
                std::async(std::launch::async, &Relocalization::loadMap, this);
            }
        } 
        else {
            HJ_ERROR("Reloc > frames exist ?: %d, map exist? : %d !!!", 
                     fs::exists("/userdata/hj/maps/map"), 
                     fs::exists("/userdata/hj/maps/point_cloud.bin"));
        }
        HJ_INFO("Reloc > reloclization start successfull!!!");
        HJ_ERROR("Reloc > reloclization start successfull!!!");
    }
    void Relocalization::stop()
    {
        HJ_INFO("Reloc > reloclization stop start!!!");
        HJ_ERROR("Reloc > reloclization stop start!!!");
        _start = false;
        _clear_data = true;
        HJ_INFO("Reloc > reloclization stop start!!!");
        HJ_ERROR("Reloc > reloclization stop start!!!");

    }
    void Relocalization::clearData()
    {
        HJ_INFO("Reloc > reloclization clearData start!!!");
        HJ_ERROR("Reloc > reloclization clearData start!!!");
        std::unique_lock<std::mutex> sonar_lock(_mutex_sonar);
        std::unique_lock<std::mutex> odom_lock(_mutex_odom);
        _start = false;
        _is_received_stop_signal.store(true);
        _poses.clear();
        _sonar_wave_datas.clear();
        _pose_timestamp.clear();
        _sonar_wave_timestamp.clear();

        _pose_reloc_source.clear();
        _sonar_wave_datas_reloc_source.clear();
        _pose_timestamp_reloc_source.clear();
        _sonar_wave_timestamp_reloc_source.clear();

        _pose_reloc_second.clear();
        _sonar_wave_datas_reloc_second.clear();
        _pose_timestamp_reloc_second.clear();
        _sonar_wave_timestamp_reloc_second.clear();

        _first_lap_end_time = 0;

        _p_sonarindedx_poseindex.clear();
        _p_sonarindedx_poseindex_reloc.clear();

        _p_sonarindex_pose.clear();
        _p_sonarindex_pose_reloc.clear();
        _clustered_poses.clear();
        _p_cloud_pose.clear();

        _sc_manager_ptr = std::make_shared<SCManager>();

        _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);

        _first_lap_end = false;

        _reloc_distance = 0;

        _first_pose = true;
        _is_enough = false;
        _target_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _icp_result_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);

        _reloc_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        _map_size = 0;

        _reloc_result.setIdentity();
        _entry_position_result.setIdentity();
        _get_source_end_pose = false;
        _source_end_pose.setIdentity();
        _get_start_pose = false;
        _start_pose.setIdentity();
        _pub_result = false;
        is_reloc_success_ = false;
        is_reloc_failed_ = false;
        _num_point_failed = 0;
        HJ_INFO("Reloc > reloclization clearData successfull!!!");
        HJ_ERROR("Reloc > reloclization clearData successfull!!!");
    }
    void Relocalization::loadOptiResults()
    {
        std::string pose_file_names = "./opti_result.txt";
        std::ifstream infile(pose_file_names.c_str());
        for (std::string line; std::getline(infile, line);)
        {
            std::stringstream ss(line);
            std::string str;
            std::vector<std::string> line_array;
            std::vector<double> vdata;
            double timestamp;
            while (getline(ss, str, ' '))
            {
                line_array.push_back(str);
            }
            for (std::size_t i = 0; i < line_array.size(); i++)
            {
                std::stringstream ss;
                double temp_data;
                ss << line_array[i];
                ss >> temp_data;
                if (i == 0)
                {
                    temp_data = temp_data * 1e-3;
                    timestamp = temp_data;
                }
                vdata.push_back(temp_data);
            }
            if (vdata.size() != 11)
                continue;
            Eigen::Quaterniond quaternion(vdata[10], vdata[7], vdata[8], vdata[9]);
            quaternion = quaternion.normalized();
            Eigen::Matrix3d R = quaternion.toRotationMatrix();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            Eigen::Vector3d t(vdata[4], vdata[5], vdata[6]);
            T.block<3, 3>(0, 0) = R;
            T.block<3, 1>(0, 3) = t;
            // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
            Eigen::Vector3d eulerAngle = R2ypr(R);

            common_data::PoseData pose;
            pose.euler_ = eulerAngle;
            pose.p_ = Eigen::Vector3d(vdata[4], vdata[5], vdata[6]);
            pose.time_ = vdata[0];
            _poses.push_back(pose);
            std::pair<int, double> tmp_pair;
            tmp_pair.first = _poses.size() - 1;
            tmp_pair.second = vdata[0];
            _pose_timestamp.push_back(tmp_pair);
            Eigen::Vector4d point(vdata[1], vdata[2], vdata[3], 1);
            double base_x = point.x();
            double base_y = point.y();
            double sonar_x, sonar_y;
            double delta_x = base_x - _left_front_x;
            double delta_y = base_y - _left_front_y;
            sonar_x = delta_x * cos(_left_front_yaw) + delta_y * sin(_left_front_yaw);
            sonar_y = -delta_x * sin(_left_front_yaw) + delta_y * cos(_left_front_yaw);
            double theta_rad = 0.0 / 180.0 * M_PI;
            float length = sonar_x / cos(theta_rad);
            std::vector<double> data_sonar;
            data_sonar.push_back(vdata[0]);
            data_sonar.push_back(length);
            data_sonar.push_back(length);

            common_data::LeftTofData tof_data;
            length = length * 1000;
            if (length > std::numeric_limits<uint32_t>::max() || length < 0)
                continue;
            tof_data.front_distance_ = static_cast<uint32_t>(length);
            tof_data.back_distance_ = static_cast<uint32_t>(length);
            tof_data.time_ = vdata[0];
            _sonar_wave_datas.push_back(tof_data);
            std::pair<int, double> tmp_pair_sonar;
            tmp_pair_sonar.first = _sonar_wave_datas.size() - 1;
            tmp_pair_sonar.second = data_sonar[0];
            _sonar_wave_timestamp.push_back(tmp_pair_sonar);
        }
        buildMultiFrame();
        saveMap();
    }
    void Relocalization::pubFailed()
    {

        HJ_INFO("relocalization num > _map->points.size() !!!\n");
        // std::string dir_to_remove = "/userdata/hj/maps/map";
        // std::string dir_to_move = "/userdata/hj/maps/map1";
        // copy_directory(dir_to_remove, dir_to_move);
        // if (boost::filesystem::exists(dir_to_remove))
        // {
        //     boost::filesystem::remove_all(dir_to_remove);
        //     std::cout << "Directory and its contents have been removed successfully." << std::endl;
        // }
        // std::string map_path = "/userdata/hj/maps/point_cloud.bin";
        // if (boost::filesystem::exists(map_path))
        // {
        //     // 删除文件
        //     boost::filesystem::remove(map_path);
        //     HJ_INFO("Reloc > map %s file has been removed successfully.", map_path.c_str());
        // }
        is_reloc_failed_ = true;
        hj_bf::HJClient client = hj_bf::HJCreateClient<hj_interface::RelocalizationResult>(
            "/relocalization_result_service");
        hj_interface::RelocalizationResult reloc_result;
        reloc_result.request.relocalization_result = 2;
        reloc_result.request.building_frames_result = 0;
        reloc_result.request.task_id = _task_id;
        client.call(reloc_result);
        HJ_INFO("Reloc > relocalization failed, pub srv success !!!\n");
    }

    void Relocalization::getNonZeroIndices(Eigen::MatrixXd &mat, std::vector<std::pair<int, int>> &nonZeroIndices, std::vector<double> &nonValue)
    {

        // 遍历矩阵以查找非零元素
        for (int i = 0; i < mat.rows(); ++i)
        {
            for (int j = 0; j < mat.cols(); ++j)
            {
                if (mat(i, j) != 0)
                {
                    nonZeroIndices.emplace_back(i, j); // 存储非零元素的索引
                    nonValue.emplace_back(mat(i, j));
                }
            }
        }
    }
    void Relocalization::loadDesc()
    {
        std::string desc_file_names = "/userdata/hj/maps/map/desc.csv";
        std::ifstream infile(desc_file_names.c_str());
        _nonZeroIndices.clear();
        _nonValue.clear();
        int n = 0;
        for (std::string line; std::getline(infile, line);)
        {
            std::stringstream ss(line);
            std::string str;
            std::vector<std::string> line_array;
            std::vector<double> data;
            while (getline(ss, str, ' '))
            {
                line_array.push_back(str);
            }
            for (std::size_t i = 0; i < line_array.size(); i++)
            {
                std::stringstream ss;
                double temp_data;
                ss << line_array[i];
                ss >> temp_data;
                data.push_back(temp_data);
            }
            if (n % 2 == 0)
            {
                _nonZeroIndices.push_back(data);
            }
            else
            {
                _nonValue.push_back(data);
            }
            n++;
        }
        // _sc_desc.clear();

        // // row 50 col 360
        // auto start_desc = std::chrono::high_resolution_clock::now();
        // Eigen::MatrixXd mat(50, 360);
        // for (std::size_t i = 0; i < _nonZeroIndices.size(); i++)
        // {
        //     for (std::size_t j = 0; j < _nonZeroIndices[i].size() - 1; j = j + 2)
        //     {
        //         mat(_nonZeroIndices[i][j], _nonZeroIndices[i][j + 1]) = _nonValue[i][(j / 2)];
        //     }
        //     _sc_desc.push_back(mat);
        // }
        // auto end_desc = std::chrono::high_resolution_clock::now();
        // auto duration_desc = std::chrono::duration_cast<std::chrono::milliseconds>(end_desc - start_desc);
        // HJ_INFO("Reloc > desc: %ld milliseconds to run\n", duration_desc.count());
    }
    void Relocalization::judgePointNum()
    {
        int num = 0;
        {
            std::unique_lock<std::mutex> lock(_mutex_sonar);
            num = _num_point_failed;
        }
        HJ_INFO("num: %d\n", num);
        HJ_INFO("map num: %zu\n", _map->points.size());
        if (num > 1.5 * (_map->points.size()))
        {
            pubFailed();
            _has_scancontext = false;
        }
    }

    void Relocalization::copy_directory(const boost::filesystem::path &source, const boost::filesystem::path &destination)
    {
        try
        {
            // 如果源路径不存在或不是目录，抛出异常
            if (!boost::filesystem::exists(source) || !fs::is_directory(source))
            {
                throw std::runtime_error("Source directory does not exist or is not a directory.");
            }

            // 如果目标路径不存在，创建目标目录
            if (!boost::filesystem::exists(destination))
            {
                boost::filesystem::create_directory(destination);
            }

            // 遍历源目录中的内容
            for (const boost::filesystem::directory_entry &entry : boost::filesystem::directory_iterator(source))
            {
                const boost::filesystem::path &sourcePath = entry.path();
                boost::filesystem::path destinationPath = destination / sourcePath.filename();

                if (boost::filesystem::is_directory(sourcePath))
                {
                    // 如果是目录，递归调用复制函数
                    copy_directory(sourcePath, destinationPath);
                }
                else if (boost::filesystem::is_regular_file(sourcePath))
                {
                    // 如果是文件，复制文件到目标位置
                    boost::filesystem::copy_file(sourcePath, destinationPath, fs::copy_option::overwrite_if_exists);
                }
                else
                {
                    std::cerr << "Skipping non-regular file or directory: " << sourcePath << std::endl;
                }
            }
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
    }

    std::string Relocalization::getCurrentTimeString()
    {
        // 获取当前系统时间
        auto now = std::chrono::system_clock::now();

        // 将系统时间转换为 time_t 类型（表示从 epoch 起的时间）
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

        // 将 time_t 转换为本地时间 tm 结构
        std::tm now_tm = *std::localtime(&now_time_t);

        // 使用 stringstream 格式化时间
        std::stringstream ss;
        ss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");

        return ss.str();
    }

    bool Relocalization::isRelocThreadStopped() {
        if (_is_reloc_thread_stopped.load()) {
            return true;
        }
        return false;
    }

    bool Relocalization::isReceivedStopSignal() {
        if (_is_received_stop_signal.load()) {
            return true;
        }
        return false;
    }
    void Relocalization::DeterminePointAMapEdge(Eigen::Matrix4d& result, double& dis)
    {
            std::vector<int> nn_result;
            mypcl::PointXYZI point;
            point.x = result(0, 3);
            point.y = result(1, 3);
            _map_kd_tree_ptr->GetClosestPoint(point, nn_result, 3); // 这里取最近邻居个数为3
            double average_result_x = 0;
            double average_result_y = 0;
            for (std::size_t i = 0; i < nn_result.size(); i++)
            {
                average_result_x += _map->points[nn_result[i]].x;
                average_result_y += _map->points[nn_result[i]].y;
            }
            average_result_x /= nn_result.size();
            average_result_y /= nn_result.size();
            double distance = sqrt(pow(point.x - average_result_x, 2) + pow(point.y - average_result_y, 2));
            HJ_INFO("Reloc > DeterminePointAMapEdge average_result_x: %lf\n", average_result_x);
            HJ_INFO("Reloc > DeterminePointAMapEdge average_result_y: %lf\n", average_result_y);
            HJ_INFO("Reloc > DeterminePointAMapEdge average_result_x: %lf\n", average_result_x);
            HJ_INFO("Reloc > DeterminePointAMapEdge average_result_y: %lf\n", average_result_y);
            HJ_INFO("Reloc > DeterminePointAMapEdge distance: %lf\n", distance);
            Eigen::Vector3d direction = Eigen::Vector3d(point.x - _map_center.x, point.y - _map_center.y, 0).normalized();
            HJ_INFO("Reloc > DeterminePointAMapEdge direction x: %lf\n", direction.x());
            HJ_INFO("Reloc > DeterminePointAMapEdge direction y: %lf\n", direction.y());
            dis = distance;
            if(distance < 0.1)
            {
                result(0, 3) = result(0, 3) - direction.x() * 0.3;
                result(1, 3) = result(1, 3) - direction.y() * 0.3;

            }
    }

} // namespace aiper_relocalization_ns
