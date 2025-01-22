#include <ExtrinsicErrorTerm/Relocalization.h>
Relocalization::Relocalization()
{
    _ok = true;
    _leftFrontCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _leftBackCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _CloudAll.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _map.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _reloc_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _left_sonar_front_sub = hj_bf::HJSubscribe("/ulsound_chatter", 1000, &Relocalization::CallbackSonar, this);
    _odom_sub = hj_bf::HJSubscribe("/fusion_result", 1000, &Relocalization::CallbackOdom, this);
    _map_pub = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/map", 1000);
    _reloc_cloud_pub = hj_bf::HJAdvertise<sensor_msgs::PointCloud2>("/reloc_cloud", 1000);
    if (std::filesystem::exists("./map"))
    {
        _hasScancontext = true;
        loadMap();
    }
    _run = std::thread(&Relocalization::run, this);
}
Relocalization::~Relocalization()
{
    if (_run.joinable())
    {
        _run.join();
    }
}

int Relocalization::buildMap()
{
    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        // std::cout<<"i:"<<i<<std::endl;
        // std::cout<<"sonarTimeStamp"<<sonarTimeStamp<<std::fixed<<std::endl;
        // std::cout<<"_Poses[index][0]"<<_Poses[index][0] * 1e-6<<std::fixed<<std::endl;
        // std::cout<<"sonarTimeStamp - _Poses[index][0] * 1e-6:"<<sonarTimeStamp - _Poses[index][0] * 1e-6 <<std::endl;
        Sonar2cloud(SonarIndex::left_front, i, index, _leftFrontCloud);
        Sonar2cloud(SonarIndex::left_back, i, index, _leftBackCloud);
    }
    std::cout << "_leftFrontCloud->points.size():" << _leftFrontCloud->points.size() << std::endl;
    std::cout << "_leftBackCloud->points.size():" << _leftBackCloud->points.size() << std::endl;
    // _leftFrontCloud->height = 1;
    // _leftFrontCloud->width = _leftFrontCloud->points.size();
    // _leftBackCloud->height = 1;
    // _leftBackCloud->width = _leftBackCloud->points.size();
    // _CloudAll->height = 1;
    // _CloudAll->width = _CloudAll->points.size();
    if (_leftFrontCloud->points.size() != 0)
        mypcl::savePLYFileBinary("leftFrontCloud.ply", *_leftFrontCloud);
    if (_leftBackCloud->points.size() != 0)
        mypcl::savePLYFileBinary("leftBackCloud.ply", *_leftBackCloud);
    if (_CloudAll->points.size() != 0)
    {
        mypcl::savePLYFileBinary("CloudAll.ply", *_CloudAll);
    }
}

int Relocalization::timeStampSynchronization(double sonarWaveTimeStamp)
{

    double pose_stamp = _PoseTimeStamp.front().second * 1e-6;

    int index = _PoseTimeStamp.size() - 1;
    int index_l = 0;
    int index_r = 0;
    bool picked = false;
    bool picked_l = false;
    bool picked_r = false;

    // 1. pick index
    for (int i = index; i >= 0; i--)
    {

        double timestamp = _PoseTimeStamp.at(i).second * 1e-6;
        if (timestamp - sonarWaveTimeStamp > 0)
        {
            picked_r = true;
            index_r = i;
        }
        if (timestamp - sonarWaveTimeStamp <= 0)
        {
            picked_l = true;
            index_l = i;
        }

        if (picked_r && picked_l)
        {
            if ((_PoseTimeStamp.at(index_r).second * 1e-6 - sonarWaveTimeStamp) >= (sonarWaveTimeStamp - _PoseTimeStamp.at(index_l).second * 1e-6))
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
    //     for (int i = 0; i <= index; i++)
    //     {
    //         _PoseTimeStamp.pop_front();
    //     }
    // }

    return _PoseTimeStamp.at(index).first;
}

int Relocalization::timeStampSynchronizationRelocSource(double sonarWaveTimeStamp)
{
    double pose_stamp = _PoseTimeStampRelocSource.front().second * 1e-6;
    int index = _PoseTimeStampRelocSource.size() - 1;
    int index_l = 0;
    int index_r = 0;
    bool picked = false;
    bool picked_l = false;
    bool picked_r = false;
    // 1. pick index
    for (int i = index; i >= 0; i--)
    {

        double timestamp = _PoseTimeStampRelocSource.at(i).second * 1e-6;

        if ((timestamp - sonarWaveTimeStamp) > 0)
        {
            picked_r = true;
            index_r = i;
        }
        if ((timestamp - sonarWaveTimeStamp) <= 0)
        {
            picked_l = true;
            index_l = i;
        }

        if (picked_r && picked_l)
        {
            if ((_PoseTimeStampRelocSource.at(index_r).second * 1e-6 - sonarWaveTimeStamp) >= (sonarWaveTimeStamp - _PoseTimeStampRelocSource.at(index_l).second * 1e-6))
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
    //     for (int i = 0; i <= index; i++)
    //     {
    //         _PoseTimeStamp.pop_front();
    //     }
    // }

    return _PoseTimeStampRelocSource.at(index).first;
}

int Relocalization::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
{
    float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
    float length;
    if (index == SonarIndex::left_front)
    {
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatas[indexSonar][1];
    }
    else if (index == SonarIndex::left_back)
    {
        fov_rad = _leftBackFovRad;
        sonar_base_x = _leftBackX;
        sonar_base_y = _leftBackY;
        sonar_base_yaw = _leftBackYaw;
        length = _SonarWaveDatas[indexSonar][2];
    }
    float half_fov = fov_rad / 2.0;
    float theta_rad = 0.0 / 180.0 * M_PI;

    // sonar坐标系的数据
    float sonar_x, sonar_y, sonar_z;
    sonar_z = 0.0;
    sonar_x = length * cos(theta_rad);
    sonar_y = length * sin(theta_rad);

    // sonar坐标系转base_link
    float base_x = sonar_base_x + (sonar_x * cos(sonar_base_yaw) - sonar_y * sin(sonar_base_yaw));
    float base_y = sonar_base_y + (sonar_x * sin(sonar_base_yaw) + sonar_y * cos(sonar_base_yaw));
    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[indexPose][6], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(_Poses[indexPose][5], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(_Poses[indexPose][4], Eigen::Vector3d::UnitX());
    cur_Q_.normalize();
    T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
    T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[indexPose][1], _Poses[indexPose][2], _Poses[indexPose][3]);
    Eigen::Vector4d p_ = Eigen::Vector4d(base_x, base_y, 0, 1);
    Eigen::Vector4d p_w = T_wc * p_;
    Eigen::Vector3d p_w_3 = Eigen::Vector3d(p_w[0], p_w[1], p_w[2]);
    if (index == SonarIndex::left_front)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 255;
        // p_rgb.g = 0;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 0;
        // p_rgb.g = 255;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
int Relocalization::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12)
{
    Eigen::Matrix4d T_12 = Eigen::Matrix4d::Identity();
    T_12.block<3, 3>(0, 0) = R12;
    T_12.block<3, 1>(0, 3) = t12;
    float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
    float length;
    if (index == SonarIndex::left_front)
    {
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatas[indexSonar][1];
    }
    else if (index == SonarIndex::left_back)
    {
        fov_rad = _leftBackFovRad;
        sonar_base_x = _leftBackX;
        sonar_base_y = _leftBackY;
        sonar_base_yaw = _leftBackYaw;
        length = _SonarWaveDatas[indexSonar][2];
    }
    float half_fov = fov_rad / 2.0;
    float theta_rad = 0.0 / 180.0 * M_PI;

    // sonar坐标系的数据
    float sonar_x, sonar_y, sonar_z;
    sonar_z = 0.0;
    sonar_x = length * cos(theta_rad);
    sonar_y = length * sin(theta_rad);
    Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);

    Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[indexPose][6], Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(_Poses[indexPose][5], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(_Poses[indexPose][4], Eigen::Vector3d::UnitX());
    cur_Q_.normalize();
    T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
    T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[indexPose][1], _Poses[indexPose][2], _Poses[indexPose][3]);

    Eigen::Vector3d p_w_3;

    if (_leftBackBase)
    {
        Eigen::Matrix4d T_base_back;
        T_base_back.setIdentity();
        Eigen::Quaterniond q_base_back = Eigen::AngleAxisd(_leftBackYaw, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        q_base_back.normalize();
        T_base_back.block<3, 3>(0, 0) = q_base_back.toRotationMatrix();
        T_base_back.block<3, 1>(0, 3) = Eigen::Vector3d(_leftBackX, _leftBackY, 0);

        Eigen::Matrix4d T_w_base = T_wc;
        Eigen::Matrix4d T_w_back = T_w_base * T_base_back;
        Eigen::Quaterniond q_w_back(T_w_back.block<3, 3>(0, 0));
        Eigen::Vector3d t_w_back(T_w_back.block<3, 1>(0, 3));
        Eigen::Vector4d p_w_4;
        p_w_4 = T_w_back * T_12 * sonar_p;
        p_w_3.x() = p_w_4.x();
        p_w_3.y() = p_w_4.y();
        p_w_3.z() = p_w_4.z();
    }
    else
    {
        Eigen::Matrix4d T_base_front;
        T_base_front.setIdentity();
        Eigen::Quaterniond q_base_front = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
        T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

        Eigen::Matrix4d T_w_base = T_wc;
        Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
        Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
        Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
        Eigen::Vector4d p_w_4;
        p_w_4 = T_w_front * T_12 * sonar_p;
        p_w_3.x() = p_w_4.x();
        p_w_3.y() = p_w_4.y();
        p_w_3.z() = p_w_4.z();

        Eigen::Matrix4d T_base_back = T_base_front * T_12;
        Eigen::Vector3d ypr;
        Eigen::Matrix3d R_base_back = T_base_back.block<3, 3>(0, 0);
        ypr = R_base_back.eulerAngles(2, 1, 0);
        Eigen::Vector3d t_base_back = T_base_back.block<3, 1>(0, 3);
        //  std::cout<<"R_base_back:"<<ypr<<std::endl;
        //  std::cout<<"t_base_back:"<<t_base_back<<std::endl;
        //  std::cout<<"R12:"<<R12<<std::endl;
    }

    if (index == SonarIndex::left_front)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 255;
        // p_rgb.g = 0;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    else if (index == SonarIndex::left_back)
    {
        mypcl::PointXYZI p;
        p.x = p_w_3[0];
        p.y = p_w_3[1];
        p.z = p_w_3[2];
        p.intensity = 0;
        cloud->points.push_back(p);
        mypcl::PointXYZI p_rgb;
        p_rgb.x = p_w_3[0];
        p_rgb.y = p_w_3[1];
        p_rgb.z = p_w_3[2];
        // p_rgb.r = 0;
        // p_rgb.g = 255;
        // p_rgb.b = 0;
        _CloudAll->points.push_back(p_rgb);
    }
    return 1;
}
void Relocalization::buildMultiFrame()
{
    std::cout << "buildMultiFrame" << std::endl;
    for (int i = 0; i < _SonarWaveDatas.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatas[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronization(sonarTimeStamp);
        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_Poses[index][6], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(_Poses[index][5], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(_Poses[index][4], Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_Poses[index][1], _Poses[index][2], _Poses[index][3]);
        _p_sonarindedx_poseindex.push_back(std::pair<int, int>(i, index));
        _p_sonarindex_pose.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));

        float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
        float length;
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatas[i][1];
        float half_fov = fov_rad / 2.0;
        float theta_rad = 0.0 / 180.0 * M_PI;

        // sonar坐标系的数据
        float sonar_x, sonar_y, sonar_z;
        sonar_z = 0.0;
        sonar_x = length * cos(theta_rad);
        sonar_y = length * sin(theta_rad);
        Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
        sonar_p = T_wc * sonar_p;
        mypcl::PointXYZI p;
        p.x = sonar_p[0];
        p.y = sonar_p[1];
        p.z = sonar_p[2];
        _map->points.push_back(p);
    }

    std::cout << "_p_sonarindedx_poseindex.size():" << _p_sonarindedx_poseindex.size() << std::endl;
    bool first_pose = false;
    Eigen::Matrix4d first_T;
    clusterPoses(_distance_threshold);

    if (_clustered_poses.size() > 1)
    {
        double travelDistance = 0.0;
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &lastGroup = _clustered_poses.back();

        // 计算最后一组中位姿依次行走的距离
        for (size_t i = 1; i < lastGroup.size(); ++i)
        {
            travelDistance += calculateDistance(lastGroup[i - 1].second.second, lastGroup[i].second.second);
        }

        // 如果累计行走的距离小于阈值，则合并至前一组
        std::cout << "travelDistance:" << travelDistance << std::endl;
        if (travelDistance < _distance_threshold * 0.7)
        {
            std::cout << "travelDistance < distance_threshold * 0.7" << std::endl;
            std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &secondLastGroup = _clustered_poses[_clustered_poses.size() - 2];
            secondLastGroup.insert(secondLastGroup.end(), lastGroup.begin(), lastGroup.end());
            _clustered_poses.pop_back(); // 移除最后一组
        }
    }

    std::cout << "multiFrameCombined" << std::endl;
    multiFrameCombined();
}
void Relocalization::multiFrameCombined()
{

    for (int i = 0; i < _clustered_poses.size(); i++)
    {
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &cluster = _clustered_poses[i];
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
        for (int j = 0; j < cluster.size(); j++)
        {
            int index = cluster[j].first;
            Eigen::Matrix4d T_wc = cluster[j].second.second;

            float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
            float length;
            fov_rad = _leftFrontFovRad;
            sonar_base_x = _leftFrontX;
            sonar_base_y = _leftFronty;
            sonar_base_yaw = _leftFrontyaw;
            length = _SonarWaveDatas[index][1];
            float half_fov = fov_rad / 2.0;
            float theta_rad = 0.0 / 180.0 * M_PI;

            // sonar坐标系的数据
            float sonar_x, sonar_y, sonar_z;
            sonar_z = 0.0;
            sonar_x = length * cos(theta_rad);
            sonar_y = length * sin(theta_rad);
            Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
            Eigen::Matrix4d T_base_front;
            T_base_front.setIdentity();
            Eigen::Quaterniond q_base_front = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
            T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
            T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

            Eigen::Matrix4d T_w_base = T_wc;
            Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
            Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
            Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
            Eigen::Vector4d p_w_4;
            Eigen::Vector4d p_0; // 转到第一帧坐标系下
            p_w_4 = T_w_front * sonar_p;
            p_0 = _clustered_poses[i][0].second.second.inverse() * p_w_4;
            mypcl::PointXYZI p;
            p.x = p_0[0];
            p.y = p_0[1];
            p.z = p_0[2];
            cloud->points.push_back(p);
        }
        // cloud->height = 1;
        // cloud->width = cloud->points.size();
        // if (cloud->points.size() != 0)
        // {
        //     pcl::io::savePLYFileBinary(std::to_string(i) + ".ply", *cloud);
        // }
        std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>> p_cloud_time_pose;
        // p_cloud_time_pose.first = cloud;
        p_cloud_time_pose.first.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::copyPointCloud(*cloud, *p_cloud_time_pose.first);
        p_cloud_time_pose.second.first = _clustered_poses[i][0].second.first;
        p_cloud_time_pose.second.second = _clustered_poses[i][0].second.second;
        _p_cloud_pose.push_back(p_cloud_time_pose);
    }
    // 新建 _p_cloud_pose_combined 来保存组合后的点云和位姿信息
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> _p_cloud_pose_combined;

    // 准备将 _p_cloud_pose 分成若干组，每组10个
    for (size_t i = 0; i < _p_cloud_pose.size(); i++)
    {
        int count = 0;
        auto combined_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

        // 定义组内时间戳和位姿的初始值
        double earliest_timestamp = _p_cloud_pose[i].second.first;
        Eigen::Matrix4d reference_pose = _p_cloud_pose[i].second.second;
        int j = i;
        Eigen::Matrix4d T1;
        while (count < _combineFrame && j < _p_cloud_pose.size())
        {
            // 累加当前组内所有帧的点到 combined_cloud
            auto new_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
            mypcl::copyPointCloud(*(_p_cloud_pose[j].first), *new_cloud);
            Eigen::Matrix4d new_pose = _p_cloud_pose[j].second.second;
            if (count == 0)
            {
                T1 = new_pose;
            }
            mypcl::transformPointCloud(*new_cloud, *new_cloud, new_pose.cast<float>());

            *combined_cloud += *(new_cloud);
            j++;
            count++;
        }

        mypcl::transformPointCloud(*combined_cloud, *combined_cloud, T1.inverse().cast<float>());
        std::ofstream out("./map/kposes.csv", std::ios::app);
        out << T1(0, 0) << " " << T1(0, 1) << " " << T1(0, 2) << " " << T1(0, 3) << " " << T1(1, 0) << " " << T1(1, 1) << " " << T1(1, 2) << " " << T1(1, 3) << " " << T1(2, 0) << " " << T1(2, 1) << " " << T1(2, 2)
            << " " << T1(2, 3) << " " << T1(3, 0) << " " << T1(3, 1) << " " << T1(3, 2) << " " << T1(3, 3) << std::endl;

        // 如果这是最后一组且它不足10帧，则与之前的点云合并
        if (count < _combineFrame / 2 && !_p_cloud_pose_combined.empty())
        {
            *(_p_cloud_pose_combined.back().first) += *combined_cloud;
        }
        else
        {
            // 存储当前组的组合点云和第一帧的时间戳及位姿
            std::pair<double, Eigen::Matrix4d> time_pose = std::make_pair(earliest_timestamp, reference_pose);
            _p_cloud_pose_combined.push_back(std::make_pair(combined_cloud, time_pose));
        }

        if (combined_cloud->points.size() != 0)
        {
            // mypcl::savePLYFileBinary(std::to_string(i) + ".ply", *combined_cloud);
        }
    }

    _p_cloud_pose = _p_cloud_pose_combined;
}

double Relocalization::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    return std::sqrt((pose1(0, 3) - pose2(0, 3)) * (pose1(0, 3) - pose2(0, 3)) +
                     (pose1(1, 3) - pose2(1, 3)) * (pose1(1, 3) - pose2(1, 3)));
}
void Relocalization::clusterPoses(double maxDistance)
{
    std::vector<bool> clustered(_p_sonarindex_pose.size(), false); // 标记位姿是否已经被分到某个簇中

    for (size_t i = 0; i < _p_sonarindex_pose.size(); ++i)
    {
        // 跳过已经聚类的位姿
        if (clustered[i])
            continue;

        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> cluster;
        std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
        temp_pair.first = _p_sonarindex_pose[i].first;
        temp_pair.second = std::pair<double, Eigen::Matrix4d>(_Poses[_p_sonarindedx_poseindex[i].second][0], _p_sonarindex_pose[i].second);
        cluster.push_back(temp_pair);
        clustered[i] = true;
        int timestamp_index = _p_sonarindedx_poseindex[i].second;
        double timestamp = _Poses[timestamp_index][0] * 1e-6;

        // 对后续位姿进行检查，判断是否与当前位姿是同一簇的一部分
        for (size_t j = i + 1; j < _p_sonarindex_pose.size(); ++j)
        {
            if (!clustered[j] && calculateDistance(_p_sonarindex_pose[i].second, _p_sonarindex_pose[j].second) <= maxDistance &&
                std::abs(timestamp - _Poses[_p_sonarindedx_poseindex[j].second][0] * 1e-6) <= 40)
            {
                temp_pair.first = _p_sonarindex_pose[j].first;
                temp_pair.second = std::pair<double, Eigen::Matrix4d>(_Poses[_p_sonarindedx_poseindex[j].second][0], _p_sonarindex_pose[j].second);
                cluster.push_back(temp_pair);
                clustered[j] = true;
            }
        }

        _clustered_poses.push_back(cluster);
    }
}
void Relocalization::buildRelocSource()
{
    for (int i = 0; i < _SonarWaveDatasRelocSource.size(); i++)
    {
        std::vector<double> data = _SonarWaveDatasRelocSource[i];
        double sonarTimeStamp = data[0] * 1e-6;
        int index = timeStampSynchronizationRelocSource(sonarTimeStamp);
        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(_PosesRelocSource[index][6], Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(_PosesRelocSource[index][5], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(_PosesRelocSource[index][4], Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(_PosesRelocSource[index][1], _PosesRelocSource[index][2], _PosesRelocSource[index][3]);
        _p_sonarindedx_poseindex_reloc.push_back(std::pair<int, int>(i, index));
        _p_sonarindex_pose_reloc.push_back(std::pair<int, Eigen::Matrix4d>(i, T_wc));
    }

    while (_sourceStartIndex > _p_sonarindex_pose_reloc.size())
    {
        _sourceStartIndex = _sourceStartIndex / 2;
    }

    std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> cluster;
    std::pair<int, std::pair<double, Eigen::Matrix4d>> temp_pair;
    temp_pair.first = _p_sonarindex_pose_reloc[_sourceStartIndex].first;
    temp_pair.second = std::pair<double, Eigen::Matrix4d>(_PosesRelocSource[_p_sonarindedx_poseindex_reloc[_sourceStartIndex].second][0], _p_sonarindex_pose_reloc[_sourceStartIndex].second);
    cluster.push_back(temp_pair);
    int timestamp_index = _p_sonarindedx_poseindex_reloc[_sourceStartIndex].second;
    double timestamp = _PosesRelocSource[timestamp_index][0] * 1e-6;

    double sum_distance = 0;
    for (size_t j = _sourceStartIndex + 1; j < _p_sonarindex_pose_reloc.size(); ++j)
    {
        double distance = calculateDistance(_p_sonarindex_pose_reloc[j - 1].second, _p_sonarindex_pose_reloc[j].second);
        sum_distance += distance;
        // if (calculateDistance(_p_sonarindex_pose_reloc[_sourceStartIndex].second, _p_sonarindex_pose_reloc[j].second) <= _distance_threshold * _combineFrame &&
        //     std::abs(timestamp - _PosesRelocSource[_p_sonarindedx_poseindex_reloc[j].second][0] * 1e-6) <= 40)
        if (sum_distance <= 7.5)

        {
            temp_pair.first = _p_sonarindex_pose_reloc[j].first;
            temp_pair.second = std::pair<double, Eigen::Matrix4d>(_PosesRelocSource[_p_sonarindedx_poseindex_reloc[j].second][0], _p_sonarindex_pose_reloc[j].second);
            cluster.push_back(temp_pair);
        }
    }
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
    for (int j = 0; j < cluster.size(); j++)
    {
        int index = cluster[j].first;
        Eigen::Matrix4d T_wc = cluster[j].second.second;

        float fov_rad = 0.0, sonar_base_x = 0.0, sonar_base_y = 0.0, sonar_base_yaw = 0.0;
        float length;
        fov_rad = _leftFrontFovRad;
        sonar_base_x = _leftFrontX;
        sonar_base_y = _leftFronty;
        sonar_base_yaw = _leftFrontyaw;
        length = _SonarWaveDatasRelocSource[index][1];
        float half_fov = fov_rad / 2.0;
        float theta_rad = 0.0 / 180.0 * M_PI;

        // sonar坐标系的数据
        float sonar_x, sonar_y, sonar_z;
        sonar_z = 0.0;
        sonar_x = length * cos(theta_rad);
        sonar_y = length * sin(theta_rad);
        Eigen::Vector4d sonar_p(sonar_x, sonar_y, sonar_z, 1);
        Eigen::Matrix4d T_base_front;
        T_base_front.setIdentity();
        Eigen::Quaterniond q_base_front = Eigen::AngleAxisd(_leftFrontyaw, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        T_base_front.block<3, 3>(0, 0) = q_base_front.toRotationMatrix();
        T_base_front.block<3, 1>(0, 3) = Eigen::Vector3d(_leftFrontX, _leftFronty, 0);

        Eigen::Matrix4d T_w_base = T_wc;
        Eigen::Matrix4d T_w_front = T_w_base * T_base_front;
        Eigen::Quaterniond q_w_front(T_w_front.block<3, 3>(0, 0));
        Eigen::Vector3d t_w_front(T_w_front.block<3, 1>(0, 3));
        Eigen::Vector4d p_w_4;
        Eigen::Vector4d p_0; // 转到第一帧坐标系下
        p_w_4 = T_w_front * sonar_p;
        p_0 = cluster[0].second.second.inverse() * p_w_4;
        mypcl::PointXYZI p;
        p.x = p_0[0];
        p.y = p_0[1];
        p.z = p_0[2];
        cloud->points.push_back(p);
    }
    _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    mypcl::copyPointCloud(*cloud, *_reloc_source);
    if (cloud->points.size() != 0)
    {
        mypcl::savePLYFileBinary("source.ply", *cloud);
    }
}
bool Relocalization::reloc()
{
    std::lock_guard<std::mutex> sonar_lock(_mutex_sonar, std::adopt_lock);
    std::lock_guard<std::mutex> odom_lock(_mutex_odom, std::adopt_lock);
    buildRelocSource();
    for (int i = 0; i < _keyFrames.size(); i++)
    {
        auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::copyPointCloud(*(_keyFrames[i]), *cloud);
        _scManager.makeAndSaveScancontextAndKeys(*cloud);
    }
    _scManager.makeAndSaveScancontextAndKeys(*_reloc_source);
    auto detectResult = _scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if (SCclosestHistoryFrameID == -1)
        return false;
    const int prev_node_idx = SCclosestHistoryFrameID;
    const int curr_node_idx = _keyFrames.size(); // because cpp starts 0 and ends n-1
    auto target = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    std::cout << "SCclosestHistoryFrameID:" << SCclosestHistoryFrameID << std::endl;
    mypcl::copyPointCloud(*(_keyFrames[SCclosestHistoryFrameID]), *target);
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation_vector(detectResult.second, Eigen::Vector3d(0, 0, 1));
    rotation_matrix = rotation_vector.toRotationMatrix();
    T.block<3, 3>(0, 0) = rotation_matrix;
    // mypcl::transformPointCloud(*target, *target, T.cast<float>());
    mypcl::savePLYFileBinary("target.ply", *target);
    auto source_tran = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::transformPointCloud(*_reloc_source, *source_tran, T.inverse().cast<float>());
    mypcl::savePLYFileBinary("source_tran.ply", *source_tran);
    sad::Icp3d::Options options;
    options.max_iteration_ = 20;
    options.min_effective_pts_ = 20;
    options.eps_ = 1e-4;
    sad::Icp3d icp(options);
    icp.SetSource(source_tran);
    icp.SetTarget(target);
    SE3 pose;
    Eigen::Matrix4d T_init;
    T_init.setIdentity();
    Eigen::Quaterniond q_init(T_init.block<3, 3>(0, 0));
    Eigen::Vector3d t_init(T_init.block<3, 1>(0, 3));
    pose = SE3(Quatd(q_init.w(), q_init.x(), q_init.y(), q_init.z()), Vec3d(t_init.x(), t_init.y(), t_init.z()));
    bool success;
    success = icp.AlignP2Line(pose);
    Eigen::Matrix4f result_loop = pose.matrix().cast<float>();
    auto result = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::transformPointCloud(*source_tran, *result, result_loop.cast<float>());
    mypcl::savePLYFileBinary("result.ply", *result);

    // ros显示
    auto final = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    mypcl::transformPointCloud(*result, *final, _kposes[SCclosestHistoryFrameID].cast<float>());
    sensor_msgs::PointCloud2 cloud_pc2;
    mypcl2PointCloud2(cloud_pc2, final);
    _reloc_cloud_pub.publish(cloud_pc2);

    auto result_filter = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    std::cout << "result->points.size():" << result->points.size() << std::endl;
    mypcl::voxelGridFilter(*result, *result_filter, 0.05);
    std::cout << "result_filter->points.size():" << result_filter->points.size() << std::endl;
    auto target_filter = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    std::cout << "target->points.size():" << target->points.size() << std::endl;
    mypcl::voxelGridFilter(*target, *target_filter, 0.05);
    std::cout << "target_filter->points.size():" << target_filter->points.size() << std::endl;
    mypcl::savePLYFileBinary("result_filter.ply", *result_filter);
    mypcl::savePLYFileBinary("target_filter.ply", *target_filter);

    std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segments;

    segments = segmentPointCloudIntoPtrs(result_filter, 10);
    std::cout << "segments.size():" << segments.size() << std::endl;

    auto result_filter_sample = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    // for (int i = 0; i < segments.size(); i++)
    // {
    //     std::cout<<"segments[i]->points.size():"<<segments[i]->points.size()<<std::endl;
    //     result_filter_sample->points.push_back(segments[i]->points[0]);
    // }
    mypcl::voxelGridFilter(*result_filter, *result_filter_sample, 2);
    std::cout << "result_filter_sample->points.size():" << result_filter_sample->points.size() << std::endl;
    mypcl::savePLYFileBinary("result_filter_sample.ply", *result_filter_sample);

    std::shared_ptr<sad::KdTree> kdtree_result = nullptr;
    kdtree_result = std::make_shared<sad::KdTree>();
    kdtree_result->BuildTree(result_filter);
    kdtree_result->SetEnableANN();
    std::shared_ptr<sad::KdTree> kdtree_target = nullptr;
    kdtree_target = std::make_shared<sad::KdTree>();
    kdtree_target->BuildTree(target_filter);
    kdtree_target->SetEnableANN();
    double diff_max = 0;

    for (int i = 0; i < result_filter_sample->points.size(); i++)
    {
        // std::cout<<"result_filter_sample->points[i].x:"<<result_filter_sample->points[i].x<<std::endl;
        // std::cout<<"result_filter_sample->points[i].y:"<<result_filter_sample->points[i].y<<std::endl;
        std::vector<int> nn_result;
        kdtree_result->GetClosestPoint(result_filter_sample->points[i], nn_result, 50); // 这里取最近邻
        std::vector<int> nn_target;
        kdtree_target->GetClosestPoint(result_filter_sample->points[i], nn_target, 50); // 这里取最近邻

        double average_result_x = 0;
        double average_result_y = 0;
        for (int j = 0; j < nn_result.size(); j++)
        {
            average_result_x += result_filter->points[nn_result[j]].x;
            average_result_y += result_filter->points[nn_result[j]].y;
        }
        average_result_x /= nn_result.size();
        average_result_y /= nn_result.size();
        double average_target_x = 0;
        double average_target_y = 0;
        for (int j = 0; j < nn_target.size(); j++)
        {
            average_target_x += target_filter->points[nn_target[j]].x;
            average_target_y += target_filter->points[nn_target[j]].y;
        }
        average_target_x /= nn_target.size();
        average_target_y /= nn_target.size();

        // double diff_result_x = result_filter->points[nn_result[0]].x - average_result_x + result_filter->points[nn_result[1]].x - average_result_x +
        //                 result_filter->points[nn_result[2]].x - average_result_x  + result_filter->points[nn_result[3]].x - average_result_x  +
        //                 result_filter->points[nn_result[4]].x - average_result_x+ result_filter->points[nn_result[5]].x - average_result_x  +
        //                 result_filter->points[nn_result[6]].x - average_result_x  + result_filter->points[nn_result[7]].x - average_result_x  +
        //                 result_filter->points[nn_result[8]].x - average_result_x  + result_filter->points[nn_result[9]].x - average_result_x ;
        // double diff_result_y = result_filter->points[nn_result[0]].y - average_result_y + result_filter->points[nn_result[1]].y - average_result_y +
        //                 result_filter->points[nn_result[2]].y - average_result_y + result_filter->points[nn_result[3]].y - average_result_y +
        //                 result_filter->points[nn_result[4]].y - average_result_y + result_filter->points[nn_result[5]].y - average_result_y +
        //                 result_filter->points[nn_result[6]].y - average_result_y + result_filter->points[nn_result[7]].y - average_result_y +
        //                 result_filter->points[nn_result[8]].y - average_result_y + result_filter->points[nn_result[9]].y - average_result_y;
        // double diff_target_x = target_filter->points[nn_target[0]].x - average_target_x  + target_filter->points[nn_target[1]].x - average_target_x  +
        //                 target_filter->points[nn_target[2]].x - average_target_x + target_filter->points[nn_target[3]].x - average_target_x +
        //                 target_filter->points[nn_target[4]].x - average_target_x  + target_filter->points[nn_target[5]].x - average_target_x  +
        //                 target_filter->points[nn_target[6]].x - average_target_x  + target_filter->points[nn_target[7]].x - average_target_x  +
        //                 target_filter->points[nn_target[8]].x - average_target_x  + target_filter->points[nn_target[9]].x - average_target_x ;
        // double diff_target_y = target_filter->points[nn_target[0]].y - average_target_y + target_filter->points[nn_target[1]].y - average_target_y +
        //                 target_filter->points[nn_target[2]].y - average_target_y + target_filter->points[nn_target[3]].y - average_target_y +
        //                 target_filter->points[nn_target[4]].y - average_target_y + target_filter->points[nn_target[5]].y - average_target_y +
        //                 target_filter->points[nn_target[6]].y - average_target_y + target_filter->points[nn_target[7]].y - average_target_y +
        //                 target_filter->points[nn_target[8]].y - average_target_y + target_filter->points[nn_target[9]].y - average_target_y;
        double diff_result_x = 0;
        double diff_result_y = 0;
        for (int j = 0; j < nn_result.size(); j++)
        {
            diff_result_x += result_filter->points[nn_result[j]].x - result_filter->points[nn_result[0]].x;
            diff_result_y += result_filter->points[nn_result[j]].y - result_filter->points[nn_result[0]].x;
            // std::cout << "diff_result_x:" << diff_result_x << std::endl;
            // std::cout << "diff_result_y:" << diff_result_y << std::endl;
        }
        double diff_target_x = 0;
        double diff_target_y = 0;
        for (int j = 0; j < nn_target.size(); j++)
        {
            diff_target_x += target_filter->points[nn_target[j]].x - result_filter->points[nn_result[0]].x;
            diff_target_y += target_filter->points[nn_target[j]].y - result_filter->points[nn_result[0]].x;
            // std::cout << "diff_target_x:" << diff_target_x << std::endl;
            // std::cout << "diff_target_y:" << diff_target_y << std::endl;
        }

        double diff_result = diff_result_x * diff_result_x + diff_result_y * diff_result_y;
        double diff_target = diff_target_x * diff_target_x + diff_target_y * diff_target_y;
        std::cout << "i:" << i << std::endl;
        // std::cout << "diff_result:" << diff_result << std::endl;
        // std::cout << "diff_target:" << diff_target << std::endl;
        std::cout << "diff:" << diff_result / diff_target << std::endl;
        if (std::fabs(diff_result / diff_target) > diff_max)
            diff_max = std::fabs(diff_result / diff_target);

        auto nn_result_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        for (int j = 0; j < nn_result.size(); j++)
        {
            nn_result_cloud->points.push_back(result_filter->points[nn_result[j]]);
        }
        mypcl::savePLYFileBinary("nn_result_cloud" + std::to_string(i) + ".ply", *nn_result_cloud);
        auto nn_target_cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        for (int j = 0; j < nn_target.size(); j++)
        {
            nn_target_cloud->points.push_back(target_filter->points[nn_target[j]]);
        }
        mypcl::savePLYFileBinary("nn_target_cloud" + std::to_string(i) + ".ply", *nn_target_cloud);
        // Eigen::Vector3d a = ransacPolynomialFit(nn_result_cloud, 100, 0.5);
        // Eigen::Vector3d b = ransacPolynomialFit(nn_target_cloud, 100, 0.5);
        // double curvature_result = computeCurvature(a(0), a(1), result_filter_sample->points[i].x);
        // double curvature_target = computeCurvature(b(0), b(1), result_filter_sample->points[i].x);
        // // std::cout<<"curvature_result:"<<curvature_result<<std::endl;
        // // std::cout<<"curvature_target:"<<curvature_target<<std::endl;
        // std::cout << "curvature_result/curvature_target:" << curvature_result / curvature_target << std::endl;
    }
    if (std::fabs(diff_max) > 30)
    {
        std::cout << "loopclosure failed" << std::endl;
        return false;
    }
    else
    {
        std::cout << "loopclosure success" << std::endl;
        return true;
    }
}

std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> Relocalization::segmentPointCloudIntoPtrs(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud, size_t segmentSize)
{
    std::vector<mypcl::PointCloud<mypcl::PointXYZI>::Ptr> segments;
    int sampleCount = static_cast<int>(pointCloud->points.size() / segmentSize);

    for (size_t i = 0; i < pointCloud->points.size(); i += sampleCount)
    {

        size_t end = std::min(pointCloud->points.size(), i + sampleCount);
        auto segment = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();

        for (size_t j = i; j < end; ++j)
        {
            segment->points.push_back(pointCloud->points[j]);
        }
        mypcl::savePLYFileBinary("segment" + std::to_string(i) + ".ply", *segment);

        segments.push_back(segment);
    }

    return segments;
}

// 基于给定点云拟合一个最佳圆，并计算该圆的曲率
double Relocalization::computeCurvature(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud)
{
    // 此示例省略了输入有效性检查
    // 创建矩阵和向量并填充
    Eigen::MatrixXd A(pointCloud->points.size(), 3);
    Eigen::VectorXd b(pointCloud->points.size());

    for (size_t i = 0; i < pointCloud->points.size(); ++i)
    {
        A(i, 0) = pointCloud->points[i].x * 2;
        A(i, 1) = pointCloud->points[i].y * 2;
        A(i, 2) = -1.0;
        b(i) = -(pointCloud->points[i].x * pointCloud->points[i].x + pointCloud->points[i].y * pointCloud->points[i].y);
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
Eigen::Vector3d Relocalization::polynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud)
{
    Eigen::MatrixXd A(pointCloud->points.size(), 3);
    Eigen::VectorXd y(pointCloud->points.size()), x(3);

    for (size_t i = 0; i < pointCloud->points.size(); ++i)
    {
        A(i, 0) = pointCloud->points[i].x * pointCloud->points[i].x; // x^2
        A(i, 1) = pointCloud->points[i].x;                           // x
        A(i, 2) = 1.0;                                               // constant term
        y(i) = pointCloud->points[i].y;
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

// RANSAC 算法实现
Eigen::Vector3d Relocalization::ransacPolynomialFit(mypcl::PointCloud<mypcl::PointXYZI>::Ptr pointCloud, int iterations, double threshold)
{
    std::default_random_engine rng;
    std::uniform_int_distribution<size_t> dist(0, pointCloud->points.size() - 1);

    size_t bestSupport = 0;
    Eigen::Vector3d bestModel;

    for (int iter = 0; iter < iterations; ++iter)
    {
        // 1. 随机选择样本点
        std::vector<Eigen::Vector2d> samples;
        auto cloud_sample = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        while (cloud_sample->points.size() < 15)
        {
            size_t i = dist(rng);
            cloud_sample->points.push_back(pointCloud->points[i]);
        }

        // 2. 拟合多项式模型
        Eigen::Vector3d model = polynomialFit(cloud_sample);

        // 3. 计算所有点对模型的支持度
        size_t support = 0;
        for (const auto &point : cloud_sample->points)
        {
            double y_est = model[0] * point.x * point.x + model[1] * point.x + model[2];
            // std::cout << "y_est:" << y_est - point.y << std::endl;
            if (std::abs(y_est - point.y) < threshold)
            {
                support++;
            }
        }

        // 4. 更新最佳模型
        if (support > bestSupport)
        {
            bestSupport = support;
            bestModel = model;
        }
    }

    return bestModel;
}

void Relocalization::CallbackOdom(const hj_interface::PoseConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_odom);
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double yaw = msg->yaw;
    double timestmap = msg->header.stamp.toNSec() * 1e-3;
    std::vector<double> data;
    data.push_back(timestmap);
    data.push_back(x);
    data.push_back(y);
    data.push_back(z);
    data.push_back(0);
    data.push_back(0);
    data.push_back(yaw);
    if (_hasScancontext)
    {
        if (!_isEnough)
        {
            _PosesRelocSource.push_back(data);
            std::pair<int, double> tmp_pair;
            tmp_pair.first = _PosesRelocSource.size() - 1;
            tmp_pair.second = data[0];
            _PoseTimeStampRelocSource.push_back(tmp_pair);
        }
    }
    else
    {
        if (!_firstLapEnd)
        {
            _Poses.push_back(data);
            std::pair<int, double> tmp_pair;
            tmp_pair.first = _Poses.size() - 1;
            tmp_pair.second = data[0];
            _PoseTimeStamp.push_back(tmp_pair);
        }
    }
}

void Relocalization::CallbackSonar(const hj_interface::UltraConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(_mutex_sonar);
    double timestmap = msg->timestamp.toNSec() * 1e-3;
    double left_front_range = msg->mid / 1000;
    double left_back_range = msg->back / 1000;
    if (left_front_range > 1)
        return;

    std::vector<double> data;
    data.push_back(timestmap);
    data.push_back(left_front_range);
    data.push_back(left_back_range);
    if (_hasScancontext)
    {
        if (!_isEnough)
        {
            _SonarWaveDatasRelocSource.push_back(data);
            std::pair<int, double> tmp_pair;
            tmp_pair.first = _SonarWaveDatasRelocSource.size() - 1;
            tmp_pair.second = data[0];
            _SonarWaveTimeStampRelocSource.push_back(tmp_pair);
        }
    }
    else
    {
        if (!_firstLapEnd)
        {
            _SonarWaveDatas.push_back(data);
            std::pair<int, double> tmp_pair;
            tmp_pair.first = _SonarWaveDatas.size() - 1;
            tmp_pair.second = data[0];
            _SonarWaveTimeStamp.push_back(tmp_pair);
        }
    }
}

void Relocalization::run()
{
    while (_ok)
    {
        if (_hasScancontext)
        {
            if (distanceToReloc())
            {
                bool sucess = reloc();
                if (sucess)
                {
                    _ok = false;
                }
                else
                {
                    reset();
                }
            }
        }
        else
        {
            if (findFirstLapEndTime())
            {
                buildMultiFrame();
                saveMap();
                _ok = false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

bool Relocalization::findFirstLapEndTime()
{
    std::lock_guard<std::mutex> lock(_mutex_odom);
    if (_Poses.size() < 2)
        return false;
    Eigen::Vector3d startPose;
    Eigen::Vector3d endPose;
    double startTime;
    double endTime;
    std::vector<double> vstartPose = _Poses[0];
    startPose << vstartPose[1], vstartPose[2], vstartPose[3];
    startTime = vstartPose[0];
    std::vector<double> vendPose = _Poses.back();
    endPose << vendPose[1], vendPose[2], vendPose[3];
    endTime = vendPose[0];
    double distance = std::sqrt((startPose[0] - endPose[0]) * (startPose[0] - endPose[0]) + (startPose[1] - endPose[1]) * (startPose[1] - endPose[1]));
    double timeDiff = endTime * 1e-6 - startTime * 1e-6;
    std::cout << "startPose:" << startPose << std::endl;
    std::cout << "endPose:" << endPose << std::endl;
    std::cout << "endTime * 1e-6:" << std::to_string(endTime * 1e-6) << std::endl;
    std::cout << "startTime * 1e-6:" << std::to_string(startTime * 1e-6) << std::endl;
    std::cout << "distance:" << distance << std::endl;
    std::cout << "timeDiff:" << timeDiff << std::fixed << std::endl;
    if (timeDiff > 100 && distance < 0.3)
    {
        _firstLapEndTime = endTime;
        std::cout << "endTime:" << _firstLapEndTime << std::endl;
        _firstLapEnd = true;
        return true;
    }
    return false;
}

void Relocalization::saveMap()
{
    std::error_code ec;
    std::filesystem::create_directories("./map", ec);
    std::ofstream out("./map/kposes.csv", std::ios::app);
    for (int i = 0; i < _p_cloud_pose.size(); i++)
    {
        auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::copyPointCloud(*(_p_cloud_pose[i].first), *cloud);
        mypcl::savePLYFileBinary("./map/cloud_" + std::to_string(i) + ".ply", *cloud);
        std::cout << "./map/cloud_" + std::to_string(i) + ".ply" << std::endl;
        Eigen::Matrix4d T = _p_cloud_pose[i].second.second;
        out<< T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)<<" "<<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)<<" "<<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)<<" "<<T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3);
        out<<std::endl;
    }
    mypcl::savePLYFileBinary("./map/map.ply", *_map);
}

std::vector<std::string> Relocalization::getFileNamesInDirectory(const std::string &directoryPath)
{
    std::vector<std::string> fileNames;

    for (const auto &entry : std::filesystem::directory_iterator(directoryPath))
    {
        if (std::filesystem::is_regular_file(entry))
        {
            fileNames.push_back(entry.path().filename().string());
        }
    }
    return fileNames;
}
void Relocalization::loadMap()
{
    std::vector<std::string> fileNames = getFileNamesInDirectory("./map");
    for (int i = 0; i < fileNames.size(); i++)
    {
        auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
        mypcl::loadPLYFile("./map/cloud_" + std::to_string(i) + ".ply", *cloud);
        std::cout << "./map/cloud_" + std::to_string(i) + ".ply" << std::endl;
        _keyFrames.push_back(cloud);
    }
    mypcl::loadPLYFile("./map/map.ply", *_map);
    sensor_msgs::PointCloud2 cloud_pc2;
    mypcl2PointCloud2(cloud_pc2, _map);
    _map_pub.publish(cloud_pc2);

    std::string filenames = "./map/kposes.csv";
    std::ifstream infile(filenames.c_str());
    int count = 0;
    for (std::string line; std::getline(infile, line);)
    {
        std::stringstream ss(line);
        std::string str;
        std::vector<std::string> lineArray;
        std::vector<double> data;
        while (getline(ss, str, ' '))
        {
            lineArray.push_back(str);
        }
        for (int i = 0; i < lineArray.size(); i++)
        {
            std::stringstream ss;
            double temp_data;
            ss << lineArray[i];
            ss >> temp_data;
            data.push_back(temp_data);
        }
        Eigen::Matrix4d T;
        T << data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15];
        _kposes.push_back(T);
    }
}

bool Relocalization::distanceToReloc()
{
    std::lock_guard<std::mutex> lock(_mutex_odom);
    if (_PosesRelocSource.size() < 2)
        return false;

    if (_first_pose)
    {
        _first_pose = false;
        _last_pose = _PosesRelocSource[0];
        return false;
    }
    Eigen::Vector3d startPose;
    Eigen::Vector3d endPose;
    double startTime;
    double endTime;

    startPose << _last_pose[1], _last_pose[2], _last_pose[3];
    startTime = _last_pose[0];
    std::vector<double> vendPose = _PosesRelocSource.back();
    endPose << vendPose[1], vendPose[2], vendPose[3];
    endTime = vendPose[0];
    double distance = std::sqrt((startPose[0] - endPose[0]) * (startPose[0] - endPose[0]) + (startPose[1] - endPose[1]) * (startPose[1] - endPose[1]));
    _relocDistance += distance;
    std::cout << "startPose:" << startPose << std::endl;
    std::cout << "endPose:" << endPose << std::endl;
    std::cout << "_relocDistance:" << _relocDistance << std::endl;

    if (_relocDistance > 10)
    {
        // if(_PosesRelocSource.back()[0] * 1e-6 - _PosesRelocSource[0][0] * 1e-6 > 100)
        // {
        _isEnough = true;
        return true;
    }
    else
    {
        _last_pose = vendPose;
        return false;
    }
}

void Relocalization::mypcl2PointCloud2(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl)
{
    std::cout << "cloud_mypcl->points.size():" << cloud_mypcl->points.size() << std::endl;
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

void Relocalization::mypcl2PointCloud2RGB(sensor_msgs::PointCloud2 &cloud_pc2, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_mypcl)
{
    std::cout << "cloud_mypcl->points.size():" << cloud_mypcl->points.size() << std::endl;
    // 设置头信息
    cloud_pc2.header.frame_id = "odom";
    cloud_pc2.header.stamp = ros::Time::now();

    // 设置点云字段
    sensor_msgs::PointCloud2Modifier modifier(cloud_pc2);
    modifier.setPointCloud2Fields(4,
                                  "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "rgb", 1, sensor_msgs::PointField::UINT32);
    modifier.resize(cloud_mypcl->points.size());

    // 直接使用迭代器填充点云数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_pc2, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_pc2, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_pc2, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_pc2, "b");
    for (size_t i = 0; i < cloud_mypcl->points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
        *iter_x = static_cast<float>(cloud_mypcl->points[i].x);
        *iter_y = static_cast<float>(cloud_mypcl->points[i].y);
        *iter_z = static_cast<float>(cloud_mypcl->points[i].z);
        // 把颜色填充进去。由于我们是用uint32打包的，所以我们只用一个迭代器即可完成rgb的填充
        uint32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));
        memcpy(&*(iter_r), &rgb, sizeof(uint32_t));
    }
}

void Relocalization::reset()
{
    std::lock_guard<std::mutex> sonar_lock(_mutex_sonar, std::adopt_lock);
    std::lock_guard<std::mutex> odom_lock(_mutex_odom, std::adopt_lock);
    _relocDistance = 0;
    _last_pose.clear();
    _first_pose = true;
    _isEnough = false;
    _reloc_cloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>());
    _SonarWaveDatasRelocSource.clear();
    _SonarWaveTimeStampRelocSource.clear();
    _p_sonarindedx_poseindex_reloc.clear();
    _p_sonarindex_pose_reloc.clear();
    _reloc_source.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _scManager.pop();

}