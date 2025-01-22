#include <ExtrinsicErrorTerm/Mapping.h>
Mapping::Mapping()
{
    _leftFrontCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _leftBackCloud.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    _CloudAll.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
    // _kdtreeFromLeftBack.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    // _kdtreeFromLeftFront.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    // _m_iNdt.setResolution(0.1);
    // _m_iNdt.setMaximumIterations(100);
    // _m_iNdt.setTransformationEpsilon(0.01);
    // _m_iNdt.setStepSize(0.01);
    // // 设置最大的对应点对距离（在此距离范围内的点对才会被用来计算变换矩阵）
    // _m_icp.setMaxCorrespondenceDistance(0.1);

    // // 设置变换矩阵的最大迭代次数
    // _m_icp.setMaximumIterations(100);

    // // 设置两次变换之间的差异的阈值，当变换足够接近上一次变换时算法将停止
    // _m_icp.setTransformationEpsilon(1e-8);

    // // 设置均方误差的阈值，当误差小于这个阈值时算法将停止
    // //_m_icp.setEuclideanFitnessEpsilon(1);
}
Mapping::~Mapping()
{
}

int Mapping::readPose(const std::string filepath)
{
    int ret = -1;
    std::string filenames = filepath + "/ArmyOdom.txt";
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
        // if (_Poses.size() > 1000)
        //     continue;
        _Poses.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = _Poses.size() - 1;
        tmp_pair.second = data[0];
        _PoseTimeStamp.push_back(tmp_pair);
    }
    return ret;
}

int Mapping::readSonarWaveData(const std::string filepath)
{
    int ret = -1;
    std::string filenames = filepath + "/ArmyUltra.txt";
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
        // if (_SonarWaveDatas.size() > 1000)
        //     continue;
        _SonarWaveDatas.push_back(data);
        std::pair<int, double> tmp_pair;
        tmp_pair.first = _SonarWaveDatas.size() - 1;
        tmp_pair.second = data[0];
        _SonarWaveTimeStamp.push_back(tmp_pair);
    }
    return ret;
}

int Mapping::buildMap()
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

int Mapping::timeStampSynchronization(double sonarWaveTimeStamp)
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
    if (index_r && !picked_l)
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

int Mapping::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
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
int Mapping::Sonar2cloud(SonarIndex index, int indexSonar, int indexPose, mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud, Eigen::Matrix3d R12, Eigen::Vector3d t12)
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
void Mapping::buildMultiFrame()
{
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
    }

    bool first_pose = false;
    Eigen::Matrix4d first_T;
    clusterPoses(_distance_threshold);
    if (_use_ori_time_pose)
        clusterPosesOri();

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

    // std::cout << "_p_sonarindex_pose: " << _p_sonarindex_pose.size() << std::endl;
    // int num_cluster;
    // for (int i = 0; i < _clustered_poses.size(); ++i)
    // {
    //     std::cout << "Cluster " << i << " has " << _clustered_poses[i].size() << " poses." << std::endl;
    //     num_cluster += _clustered_poses[i].size();
    // }
    // std::cout << "num_cluster:" << num_cluster << std::endl;
    std::cout << "multiFrameCombined" << std::endl;
    multiFrameCombined();
    if (_use_ori_time_pose)
        multiFrameCombinedOri();
}
void Mapping::multiFrameCombined()
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
    // 构造多帧点云使得每帧之间有重合
    std::vector<std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>>> p_cloud_pose_new;
    for (int i = 0; i < _p_cloud_pose.size() - 1; i++)
    {
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud1(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud2(new mypcl::PointCloud<mypcl::PointXYZI>);
        Eigen::Matrix4d T1;
        Eigen::Matrix4d T2;

        mypcl::copyPointCloud(*_p_cloud_pose[i].first, *cloud1);
        mypcl::copyPointCloud(*_p_cloud_pose[i + 1].first, *cloud2);
        T1 = _p_cloud_pose[i].second.second;
        T2 = _p_cloud_pose[i + 1].second.second;

        mypcl::transformPointCloud(*cloud1, *cloud1, T1.cast<float>());
        mypcl::transformPointCloud(*cloud2, *cloud2, T2.cast<float>());
        for (auto &p : cloud1->points)
        {
            p.intensity = 1;
        }
        for (auto &p : cloud2->points)
        {
            p.intensity = 2;
        }
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_combine(new mypcl::PointCloud<mypcl::PointXYZI>);
        cloud_combine->points.insert(cloud_combine->points.end(), cloud1->points.begin(), cloud1->points.end());
        cloud_combine->points.insert(cloud_combine->points.end(), cloud2->points.begin(), cloud2->points.end());
        Eigen::Matrix4d T = T1.inverse();
        mypcl::transformPointCloud(*cloud_combine, *cloud_combine, T.cast<float>());
        // cloud_combine->height = 1;
        // cloud_combine->width = cloud_combine->points.size();
        // if (cloud_combine->points.size() != 0)
        // {
        //     pcl::io::savePLYFileBinary(std::to_string(i) + ".ply", *cloud_combine);
        // }
        std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>> p_cloud_time_pose;
        p_cloud_time_pose.first = cloud_combine;
        p_cloud_time_pose.second.first = _p_cloud_pose[i].second.first;
        p_cloud_time_pose.second.second = T1;
        p_cloud_pose_new.push_back(p_cloud_time_pose);
    }
    _p_cloud_pose = p_cloud_pose_new;
}

void Mapping::multiFrameCombinedOri()
{

    for (int i = 0; i < _clustered_poses_ori.size(); i++)
    {
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &cluster = _clustered_poses_ori[i];
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
            p_0 = _clustered_poses_ori[i][0].second.second.inverse() * p_w_4;
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
        p_cloud_time_pose.second.first = _clustered_poses_ori[i][0].second.first;
        p_cloud_time_pose.second.second = _clustered_poses_ori[i][0].second.second;
        _p_cloud_pose_ori.push_back(p_cloud_time_pose);
    }
}

double Mapping::calculateDistance(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    return std::sqrt((pose1(0, 3) - pose2(0, 3)) * (pose1(0, 3) - pose2(0, 3)) +
                     (pose1(1, 3) - pose2(1, 3)) * (pose1(1, 3) - pose2(1, 3)));
}
void Mapping::clusterPoses(double maxDistance)
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
void Mapping::clusterPosesOri(double maxDistance)
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

        _clustered_poses_ori.push_back(cluster);
    }
    std::cout << "_clustered_poses_ori.size():" << _clustered_poses_ori.size() << std::endl;
    if (_clustered_poses_ori.size() > 1)
    {
        double travelDistance = 0.0;
        std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &lastGroup = _clustered_poses_ori.back();

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
            std::vector<std::pair<int, std::pair<double, Eigen::Matrix4d>>> &secondLastGroup = _clustered_poses_ori[_clustered_poses_ori.size() - 2];
            secondLastGroup.insert(secondLastGroup.end(), lastGroup.begin(), lastGroup.end());
            _clustered_poses_ori.pop_back(); // 移除最后一组
        }
    }
}
void Mapping::map()
{
    std::cout << "_p_cloud_pose.size():" << _p_cloud_pose.size() << std::endl;
    for (int i = 0; i < _p_cloud_pose.size(); i++)
    {
        if (_loop_index == 1)
        {
            continue;
        }
        if (_keyframes.empty())
        {
            _keyframes.push_back(_p_cloud_pose[i]);
            _keyframes_show.push_back(_p_cloud_pose[i]);
            continue;
        }
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::copyPointCloud(*_p_cloud_pose[i].first, *cloud);
        Eigen::Matrix4d T = _p_cloud_pose[i].second.second;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr localMap(new mypcl::PointCloud<mypcl::PointXYZI>);
        for (int j = 0; j < _keyframes.size(); j++)
        {
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr keyframe = _keyframes[j].first;
            Eigen::Matrix4d T_keyframe = _keyframes[j].second.second;
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr transformedCloud(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe.cast<float>());
            *localMap += *transformedCloud;
        }
        // sad::Icp3d::Options options;
        // options.max_iteration_ = 200;
        // options.min_effective_pts_ = 20;
        // options.eps_ = 1e-4;
        // // sad::Icp3d icp;
        // sad::Icp3d icp(options);
        // icp.SetSource(cloud);
        // icp.SetTarget(localMap);
        // SE3 pose_init;
        // Eigen::Matrix4d T_init = T;
        // Eigen::Quaterniond q_init(T_init.block<3, 3>(0, 0));
        // Eigen::Vector3d t_init(T_init.block<3, 1>(0, 3));
        // pose_init = SE3(Quatd(q_init.w(), q_init.x(), q_init.y(), q_init.z()), Vec3d(t_init.x(), t_init.y(), t_init.z()));
        // bool success;
        // success = icp.AlignP2Line(pose_init);

        // _m_iNdt.setInputTarget(localMap);
        // _m_iNdt.setInputSource(cloud);
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr out(new mypcl::PointCloud<mypcl::PointXYZI>());
        Eigen::Matrix4f pose;
        // pose = pose_init.matrix().cast<float>();
        pose = T.cast<float>();

        // _m_iNdt.align(*out, pose);
        // pose = _m_iNdt.getFinalTransformation();
        // pose = T.cast<float>();
        if (i < _p_cloud_pose.size() - 1)
        {
            Eigen::Matrix4d T_next = _p_cloud_pose[i + 1].second.second;
            Eigen::Matrix4d T_12 = T.inverse() * T_next;
            _p_cloud_pose[i].second.second = pose.cast<double>();
            _p_cloud_pose[i + 1].second.second = pose.cast<double>() * T_12;
        }

        float closest_d = std::numeric_limits<float>::infinity();
        for (const auto &k : _keyframes)
        {
            // calculate distance between current pose and pose in keyframes
            double delta_d = sqrt(pow(pose(0, 3) - k.second.second(0, 3), 2) + pow(pose(1, 3) - k.second.second(1, 3), 2));
            // store into variable
            if (delta_d < closest_d)
            {
                closest_d = delta_d;
            }
        }
        std::cout << "closest_d:" << closest_d << std::endl;
        std::pair<mypcl::PointCloud<mypcl::PointXYZI>::Ptr, std::pair<double, Eigen::Matrix4d>> keyframe;
        keyframe.first.reset(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::copyPointCloud(*cloud, *keyframe.first);
        // keyframe.first = cloud;
        keyframe.second.first = _p_cloud_pose[i].second.first;
        keyframe.second.second = pose.cast<double>();
        if (closest_d > 0.7 * _distance_threshold)
        {
            _keyframes.push_back(keyframe);
        }
        // _keyframes_show.push_back(_p_cloud_pose[i]);
        _keyframes_show.push_back(keyframe);

        std::cout << "pose:" << pose << std::endl;
        std::cout << "T:" << T << std::endl;

        int loop_index = -2;
        std::cout << "_p_cloud_pose[i].second.first:" << _p_cloud_pose[i].second.first << std::endl;
        if (judgeFeaturesDistribution(cloud))
            loop_index = DetectLoopClosure(pose.cast<double>(), _p_cloud_pose[i].second.first);
        // if (loop_index > -1 && !_first_loop)
        std::cout << "_p_cloud_pose[i].second.first:" << _p_cloud_pose[i].second.first << std::endl;
        if (loop_index > -1)
        {
            std::cout << "loop_index:" << loop_index << std::endl;
            _first_loop = true;
            Eigen::Matrix4d T_source = pose.cast<double>();
            Eigen::Matrix4d T_target = _keyframes[loop_index].second.second;
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr target(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::copyPointCloud(*_keyframes[loop_index].first, *target);
            // _m_iNdt.setInputTarget(target);
            // _m_iNdt.setInputSource(cloud);
            // _m_icp.setInputTarget(target);
            // _m_icp.setInputSource(cloud);
            sad::Icp3d::Options options;
            options.max_iteration_ = 200;
            options.min_effective_pts_ = 20;
            options.eps_ = 1e-4;
            // sad::Icp3d icp;
            sad::Icp3d icp(options);
            icp.SetSource(cloud);
            icp.SetTarget(target);
            SE3 pose;
            Eigen::Matrix4d T_init = T_target.inverse() * T_source;
            // Eigen::Matrix4d T_init;
            // T_init.setIdentity();
            Eigen::Quaterniond q_init(T_init.block<3, 3>(0, 0));
            Eigen::Vector3d t_init(T_init.block<3, 1>(0, 3));
            pose = SE3(Quatd(q_init.w(), q_init.x(), q_init.y(), q_init.z()), Vec3d(t_init.x(), t_init.y(), t_init.z()));
            bool success;
            success = icp.AlignP2Line(pose);
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr out_loop(new mypcl::PointCloud<mypcl::PointXYZI>());
            std::cout << "cloud->points.size():" << cloud->points.size() << std::endl;
            std::cout << "target->points.size():" << target->points.size() << std::endl;

            //_m_iNdt.align(*out_loop, (T_target.inverse() * T_source).cast<float>());
            //_m_icp.align(*out_loop, (T_target.inverse() * T_source).cast<float>());
            // Eigen::Matrix4f result_loop = _m_iNdt.getFinalTransformation();
            // Eigen::Matrix4f result_loop = _m_icp.getFinalTransformation();
            Eigen::Matrix4f result_loop = pose.matrix().cast<float>();
            *out_loop += *target;
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_transform(new mypcl::PointCloud<mypcl::PointXYZI>());
            mypcl::transformPointCloud(*cloud, *cloud_transform, result_loop.cast<float>());
            *out_loop += *cloud_transform;
            *cloud += *target;
            std::cout << "out_loop->points.size():" << out_loop->points.size() << std::endl;
            // out_loop->height = 1;
            // out_loop->width = out_loop->points.size();
            if (out_loop->points.size() != 0)
            {
                mypcl::savePLYFileBinary("out_loop.ply", *out_loop);
            }
            // cloud->height = 1;
            // cloud->width = out_loop->points.size();
            if (cloud->points.size() != 0)
            {
                mypcl::savePLYFileBinary("out_loop_ori.ply", *cloud);
            }
            std::cout << "_p_cloud_pose[i].second.first:" << _p_cloud_pose[i].second.first << std::endl;
            if (!_use_ori_time_pose)
                LoopClosure(loop_index, _keyframes_show.size() - 1, T_target, T_target * result_loop.cast<double>());
            else
                LoopClosureOriPose(_keyframes[loop_index].second.first, _p_cloud_pose[i].second.first, T_target, T_target * result_loop.cast<double>());
            _loop_index++;
        }
    }
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr out(new mypcl::PointCloud<mypcl::PointXYZI>);
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr out_show(new mypcl::PointCloud<mypcl::PointXYZI>);
    for (int j = 0; j < _keyframes.size(); j++)
    {
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr keyframe = _keyframes[j].first;
        Eigen::Matrix4d T_keyframe = _keyframes[j].second.second;
        mypcl::PointCloud<mypcl::PointXYZI>::Ptr transformedCloud(new mypcl::PointCloud<mypcl::PointXYZI>);
        mypcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe.cast<float>());
        *out += *transformedCloud;
    }
    // out->height = 1;
    // out->width = out->points.size();
    if (out->points.size() != 0)
    {
        mypcl::savePLYFileBinary("out.ply", *out);
    }
    std::cout << "_keyframes_show.size():" << _keyframes_show.size() << std::endl;
    if (!_use_ori_time_pose)
    {
        for (int j = 0; j < _keyframes_show.size(); j++)
        {
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr keyframe(new mypcl::PointCloud<mypcl::PointXYZI>);
            for (auto &p : _keyframes_show[j].first->points)
            {
                if (p.intensity == 1)
                {
                    keyframe->points.push_back(p);
                }
            }
            Eigen::Matrix4d T_keyframe = _keyframes_show[j].second.second;
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr transformedCloud(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe.cast<float>());
            *out_show += *transformedCloud;
        }
    }
    else
    {
        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {
            if (_p_cloud_pose_ori[i].second.first > _keyframes_show.back().second.first)
            {
                continue;
            }
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr keyframe(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::copyPointCloud(*_p_cloud_pose_ori[i].first, *keyframe);
            Eigen::Matrix4d T_keyframe = _p_cloud_pose_ori[i].second.second;
            mypcl::PointCloud<mypcl::PointXYZI>::Ptr transformedCloud(new mypcl::PointCloud<mypcl::PointXYZI>);
            mypcl::transformPointCloud(*keyframe, *transformedCloud, T_keyframe.cast<float>());
            *out_show += *transformedCloud;
        }
    }
    // out_show->height = 1;
    // out_show->width = out_show->points.size();
    if (out_show->points.size() != 0)
    {
        mypcl::savePLYFileBinary("out_show.ply", *out_show);
    }
}
void Mapping::LoopClosureOriPose(const double time1, const double time2, const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    int index1 = -1;
    int index2 = -1;
    // std::cout << "time1:" << time1 << std::endl;
    // std::cout << "time2:" << time2 << std::endl;
    // std::cout << "_p_cloud_pose_ori.size():" << _p_cloud_pose_ori.size() << std::endl;
    for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
    {
        if (_p_cloud_pose_ori[i].second.first == time1)
        {
            index1 = i;
        }
        if (_p_cloud_pose_ori[i].second.first == time2)
        {
            index2 = i;
        }
    }
    if (_use_ceres)
    {
        double para_t[_p_cloud_pose_ori.size()][3];
        double para_q[_p_cloud_pose_ori.size()][4];
        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {
            para_t[i][0] = _p_cloud_pose_ori[i].second.second(0, 3);
            para_t[i][1] = _p_cloud_pose_ori[i].second.second(1, 3);
            para_t[i][2] = _p_cloud_pose_ori[i].second.second(2, 3);
            Eigen::Matrix3d temp_R = _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q(temp_R);

            para_q[i][0] = temp_q.x();
            para_q[i][1] = temp_q.y();
            para_q[i][2] = temp_q.z();
            para_q[i][3] = temp_q.w();
        }

        ceres::Problem problem;
        ceres::LocalParameterization *local_parameterization = new ceres::EigenQuaternionParameterization();

        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {

            problem.AddParameterBlock(para_q[i], 4, local_parameterization);
            problem.AddParameterBlock(para_t[i], 3);
        }

        for (int i = 0; i < _p_cloud_pose_ori.size() - 1; i++)
        {
            Eigen::Vector3d para_t_constraint;
            Eigen::Vector4f para_q_constraint;
            Eigen::Matrix3d temp_R2 = _p_cloud_pose_ori[i + 1].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_2(temp_R2);
            Eigen::Vector3d temp_t_2(_p_cloud_pose_ori[i + 1].second.second(0, 3), _p_cloud_pose_ori[i + 1].second.second(1, 3), _p_cloud_pose_ori[i + 1].second.second(2, 3));
            Eigen::Matrix3d temp_R1 = _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_1(temp_R1);
            Eigen::Vector3d temp_t_1(_p_cloud_pose_ori[i].second.second(0, 3), _p_cloud_pose_ori[i].second.second(1, 3), _p_cloud_pose_ori[i].second.second(2, 3));

            Eigen::Matrix3d R_constraint;
            Eigen::Vector3d t_constraint;

            R_constraint = temp_q_2.toRotationMatrix().inverse() * temp_q_1.toRotationMatrix();
            t_constraint = temp_q_2.toRotationMatrix().inverse() * (temp_t_1 - temp_t_2);

            Eigen::Quaterniond q_constraint(R_constraint);

            para_t_constraint = t_constraint;
            para_q_constraint[0] = q_constraint.x();
            para_q_constraint[1] = q_constraint.y();
            para_q_constraint[2] = q_constraint.z();
            para_q_constraint[3] = q_constraint.w();
            ceres::CostFunction *cost_function = consecutivePose::Create(
                para_q_constraint,
                para_t_constraint);
            problem.AddResidualBlock(cost_function, NULL, para_q[i], para_t[i], para_q[i + 1], para_t[i + 1]);
        }

        Eigen::Vector3d para_t_loop;
        Eigen::Vector4f para_q_loop;
        Eigen::Matrix3d loop_R2 = pose2.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_2(loop_R2);
        Eigen::Vector3d loop_t_2(pose2(0, 3), pose2(1, 3), pose2(2, 3));
        Eigen::Matrix3d loop_R1 = pose1.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_1(loop_R1);
        Eigen::Vector3d loop_t_1(pose1(0, 3), pose1(1, 3), pose1(2, 3));

        Eigen::Matrix3d R_constraint_loop;
        Eigen::Vector3d t_constraint_loop;

        R_constraint_loop = loop_q_2.toRotationMatrix().inverse() * loop_q_1.toRotationMatrix();
        t_constraint_loop = loop_q_2.toRotationMatrix().inverse() * (loop_t_1 - loop_t_2);

        Eigen::Quaterniond q_constraint_loop(R_constraint_loop);

        para_t_loop = t_constraint_loop;
        para_q_loop[0] = q_constraint_loop.x();
        para_q_loop[1] = q_constraint_loop.y();
        para_q_loop[2] = q_constraint_loop.z();
        para_q_loop[3] = q_constraint_loop.w();
        ceres::CostFunction *cost_function = loopPose::Create(
            para_q_loop,
            para_t_loop);
        problem.AddResidualBlock(cost_function, NULL, para_q[index1], para_t[index1], para_q[index2], para_t[index2]);
        problem.SetParameterBlockConstant(para_q[index1]);
        problem.SetParameterBlockConstant(para_t[index1]);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = 20;
        options.minimizer_progress_to_stdout = true;
        options.max_solver_time_in_seconds = 600;
        options.max_num_iterations = 1000;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1)
        {
            std::cout << " converge:" << summary.final_cost << std::endl;
        }
        else
        {
            std::cout << " not converge :" << summary.final_cost << std::endl;
        }
        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {
            Eigen::Matrix3d R;
            Eigen::Quaterniond q(para_q[i][3], para_q[i][0], para_q[i][1], para_q[i][2]);
            R = q.toRotationMatrix();
            _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0) = R;
            _p_cloud_pose_ori[i].second.second.block<3, 1>(0, 3) = Eigen::Vector3d(para_t[i][0], para_t[i][1], para_t[i][2]);
        }
    }
    else
    {
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        // variable-size block solver
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
                g2o::make_unique<
                    g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

        optimizer.setAlgorithm(solver);
        // set up two poses
        int vertex_id = 0;
        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {
            // set up rotation and translation for this node
            Eigen::Vector3d t(_p_cloud_pose_ori[i].second.second(0, 3), _p_cloud_pose_ori[i].second.second(1, 3), _p_cloud_pose_ori[i].second.second(2, 3));
            Eigen::Matrix3d temp_R = _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond q(temp_R);

            Eigen::Isometry3d cam; // camera pose
            cam = q;
            cam.translation() = t;
            Sophus::SE3 SE3_T(q, t);

            // set up node
            VertexPose *vc = new VertexPose();
            vc->setEstimate(SE3_T);

            vc->setId(vertex_id); // vertex id

            // set first cam pose fixed
            if (i == 0)
                vc->setFixed(true);

            // add to optimizer
            optimizer.addVertex(vc);

            vertex_id++;
        }

        for (int i = 0; i < _p_cloud_pose_ori.size() - 1; i++)
        {
            Eigen::Vector3d para_t_constraint;
            Eigen::Vector4f para_q_constraint;
            Eigen::Matrix3d temp_R2 = _p_cloud_pose_ori[i + 1].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_2(temp_R2);
            Eigen::Vector3d temp_t_2(_p_cloud_pose_ori[i + 1].second.second(0, 3), _p_cloud_pose_ori[i + 1].second.second(1, 3), _p_cloud_pose_ori[i + 1].second.second(2, 3));
            Eigen::Matrix3d temp_R1 = _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_1(temp_R1);
            Eigen::Vector3d temp_t_1(_p_cloud_pose_ori[i].second.second(0, 3), _p_cloud_pose_ori[i].second.second(1, 3), _p_cloud_pose_ori[i].second.second(2, 3));

            Eigen::Matrix3d R_constraint;
            Eigen::Vector3d t_constraint;

            R_constraint = temp_q_2.toRotationMatrix().inverse() * temp_q_1.toRotationMatrix();
            t_constraint = temp_q_2.toRotationMatrix().inverse() * (temp_t_1 - temp_t_2);

            Eigen::Quaterniond q_constraint(R_constraint);
            Sophus::SE3 se3_1(temp_q_1, temp_t_1);
            Sophus::SE3 se3_2(temp_q_2, temp_t_2);
            Sophus::SE3 se3_constraint(q_constraint, t_constraint);
            se3_constraint = se3_2.inverse() * se3_1;
            // Sophus::SE3 SE3_constraint(q_constraint, t_constraint);
            EdgeRelativeMotion *e // new edge with correct cohort for caching
                = new EdgeRelativeMotion();
            VertexPose *vp0 =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i)->second);
            VertexPose *vp1 =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i + 1)->second);
            e->setVertex(0, vp0); // first viewpoint
            e->setVertex(1, vp1); // second viewpoint
            e->setMeasurement(se3_constraint);
            e->information() = 100 * Eigen::Matrix<double, 6, 6>::Identity();
            // use this for point-point
            //    e->information().setIdentity();

            // e->setRobustKernel(true);
            // e->setHuberWidth(0.01);

            optimizer.addEdge(e);
        }
        Eigen::Matrix3d loop_R2 = pose2.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_2(loop_R2);
        Eigen::Vector3d loop_t_2(pose2(0, 3), pose2(1, 3), pose2(2, 3));
        Eigen::Matrix3d loop_R1 = pose1.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_1(loop_R1);
        Eigen::Vector3d loop_t_1(pose1(0, 3), pose1(1, 3), pose1(2, 3));
        Sophus::SE3 se3_loop1(loop_q_1, loop_t_1);
        Sophus::SE3 se3_loop2(loop_q_2, loop_t_2);
        Sophus::SE3 se3_loop = se3_loop2.inverse() * se3_loop1;
        EdgeRelativeMotion *e // new edge with correct cohort for caching
            = new EdgeRelativeMotion();
        std::cout << "time1:" << std::to_string(time1) << std::endl;
        std::cout << "time2:" << std::to_string(time2) << std::endl;
        std::cout << "index1:" << index1 << std::endl;
        std::cout << "index2:" << index2 << std::endl;
        VertexPose *vp0 =
            dynamic_cast<VertexPose *>(optimizer.vertices().find(index1)->second);
        VertexPose *vp1 =
            dynamic_cast<VertexPose *>(optimizer.vertices().find(index2)->second);
        vp0->setFixed(true);
        e->setMeasurement(se3_loop);
        e->setVertex(0, vp0); // first viewpoint
        e->setVertex(1, vp1); // second viewpoint
        std::cout << "111" << std::endl;
        e->information() = 1000 * Eigen::Matrix<double, 6, 6>::Identity();
        optimizer.addEdge(e);
        optimizer.initializeOptimization();

        optimizer.computeActiveErrors();

        optimizer.setVerbose(true);
        std::cout << "222" << std::endl;
        optimizer.optimize(25);
        std::cout << "333" << std::endl;
        for (int i = 0; i < _p_cloud_pose_ori.size(); i++)
        {
            VertexPose *vc =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i)->second);
            Sophus::SE3 pose = vc->estimate();

            _p_cloud_pose_ori[i].second.second.block<3, 3>(0, 0) = pose.unit_quaternion().toRotationMatrix();
            _p_cloud_pose_ori[i].second.second.block<3, 1>(0, 3) = pose.translation();
        }
    }
}
void Mapping::LoopClosure(const int index1, const int index2, const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2)
{
    if (_use_ceres)
    {
        double para_t[_keyframes_show.size()][3];
        double para_q[_keyframes_show.size()][4];
        for (int i = 0; i < _keyframes_show.size(); i++)
        {
            para_t[i][0] = _keyframes_show[i].second.second(0, 3);
            para_t[i][1] = _keyframes_show[i].second.second(1, 3);
            para_t[i][2] = _keyframes_show[i].second.second(2, 3);
            Eigen::Matrix3d temp_R = _keyframes_show[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q(temp_R);

            para_q[i][0] = temp_q.x();
            para_q[i][1] = temp_q.y();
            para_q[i][2] = temp_q.z();
            para_q[i][3] = temp_q.w();
        }

        ceres::Problem problem;
        ceres::LocalParameterization *local_parameterization = new ceres::EigenQuaternionParameterization();

        for (int i = 0; i < _keyframes_show.size(); i++)
        {

            problem.AddParameterBlock(para_q[i], 4, local_parameterization);
            problem.AddParameterBlock(para_t[i], 3);
        }
        problem.SetParameterBlockConstant(para_q[0]);
        problem.SetParameterBlockConstant(para_t[0]);

        for (int i = 0; i < _keyframes_show.size() - 1; i++)
        {
            Eigen::Vector3d para_t_constraint;
            Eigen::Vector4f para_q_constraint;
            Eigen::Matrix3d temp_R2 = _keyframes_show[i + 1].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_2(temp_R2);
            Eigen::Vector3d temp_t_2(_keyframes_show[i + 1].second.second(0, 3), _keyframes_show[i + 1].second.second(1, 3), _keyframes_show[i + 1].second.second(2, 3));
            Eigen::Matrix3d temp_R1 = _keyframes_show[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_1(temp_R1);
            Eigen::Vector3d temp_t_1(_keyframes_show[i].second.second(0, 3), _keyframes_show[i].second.second(1, 3), _keyframes_show[i].second.second(2, 3));

            Eigen::Matrix3d R_constraint;
            Eigen::Vector3d t_constraint;

            R_constraint = temp_q_2.toRotationMatrix().inverse() * temp_q_1.toRotationMatrix();
            t_constraint = temp_q_2.toRotationMatrix().inverse() * (temp_t_1 - temp_t_2);

            Eigen::Quaterniond q_constraint(R_constraint);

            para_t_constraint = t_constraint;
            para_q_constraint[0] = q_constraint.x();
            para_q_constraint[1] = q_constraint.y();
            para_q_constraint[2] = q_constraint.z();
            para_q_constraint[3] = q_constraint.w();
            ceres::CostFunction *cost_function = consecutivePose::Create(
                para_q_constraint,
                para_t_constraint);
            problem.AddResidualBlock(cost_function, NULL, para_q[i], para_t[i], para_q[i + 1], para_t[i + 1]);
        }

        Eigen::Vector3d para_t_loop;
        Eigen::Vector4f para_q_loop;
        Eigen::Matrix3d loop_R2 = pose2.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_2(loop_R2);
        Eigen::Vector3d loop_t_2(pose2(0, 3), pose2(1, 3), pose2(2, 3));
        Eigen::Matrix3d loop_R1 = pose1.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_1(loop_R1);
        Eigen::Vector3d loop_t_1(pose1(0, 3), pose1(1, 3), pose1(2, 3));

        Eigen::Matrix3d R_constraint_loop;
        Eigen::Vector3d t_constraint_loop;

        R_constraint_loop = loop_q_2.toRotationMatrix().inverse() * loop_q_1.toRotationMatrix();
        t_constraint_loop = loop_q_2.toRotationMatrix().inverse() * (loop_t_1 - loop_t_2);

        Eigen::Quaterniond q_constraint_loop(R_constraint_loop);

        para_t_loop = t_constraint_loop;
        para_q_loop[0] = q_constraint_loop.x();
        para_q_loop[1] = q_constraint_loop.y();
        para_q_loop[2] = q_constraint_loop.z();
        para_q_loop[3] = q_constraint_loop.w();
        ceres::CostFunction *cost_function = loopPose::Create(
            para_q_loop,
            para_t_loop);
        problem.AddResidualBlock(cost_function, NULL, para_q[index1], para_t[index1], para_q[index2], para_t[index2]);
        problem.SetParameterBlockConstant(para_q[index1]);
        problem.SetParameterBlockConstant(para_t[index1]);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = 20;
        options.minimizer_progress_to_stdout = true;
        options.max_solver_time_in_seconds = 600;
        options.max_num_iterations = 1000;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 1)
        {
            std::cout << " converge:" << summary.final_cost << std::endl;
        }
        else
        {
            std::cout << " not converge :" << summary.final_cost << std::endl;
        }
        for (int i = 0; i < _keyframes_show.size(); i++)
        {
            Eigen::Matrix3d R;
            Eigen::Quaterniond q(para_q[i][3], para_q[i][0], para_q[i][1], para_q[i][2]);
            R = q.toRotationMatrix();
            _keyframes_show[i].second.second.block<3, 3>(0, 0) = R;
            _keyframes_show[i].second.second.block<3, 1>(0, 3) = Eigen::Vector3d(para_t[i][0], para_t[i][1], para_t[i][2]);
        }
    }
    else
    {
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        // variable-size block solver
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
                g2o::make_unique<
                    g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));

        optimizer.setAlgorithm(solver);
        // set up two poses
        int vertex_id = 0;
        for (int i = 0; i < _keyframes_show.size(); i++)
        {
            // set up rotation and translation for this node
            Eigen::Vector3d t(_keyframes_show[i].second.second(0, 3), _keyframes_show[i].second.second(1, 3), _keyframes_show[i].second.second(2, 3));
            Eigen::Matrix3d temp_R = _keyframes_show[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond q(temp_R);

            Eigen::Isometry3d cam; // camera pose
            cam = q;
            cam.translation() = t;
            Sophus::SE3 SE3_T(q, t);

            // set up node
            VertexPose *vc = new VertexPose();
            vc->setEstimate(SE3_T);
            std::cout << "i:" << i << std::endl;
            std::cout << "vertex_id：" << vertex_id << std::endl;

            vc->setId(vertex_id); // vertex id

            // set first cam pose fixed
            if (i == 0)
                vc->setFixed(true);

            // add to optimizer
            optimizer.addVertex(vc);

            vertex_id++;
        }
        std::cout << "66666666666666666666666" << std::endl;
        std::cout << "optimizer.vertices().size():" << optimizer.vertices().size() << std::endl;
        for (int i = 0; i < _keyframes_show.size() - 1; i++)
        {
            std::cout << "i:" << i << std::endl;
            Eigen::Vector3d para_t_constraint;
            Eigen::Vector4f para_q_constraint;
            Eigen::Matrix3d temp_R2 = _keyframes_show[i + 1].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_2(temp_R2);
            Eigen::Vector3d temp_t_2(_keyframes_show[i + 1].second.second(0, 3), _keyframes_show[i + 1].second.second(1, 3), _keyframes_show[i + 1].second.second(2, 3));
            Eigen::Matrix3d temp_R1 = _keyframes_show[i].second.second.block<3, 3>(0, 0);
            Eigen::Quaterniond temp_q_1(temp_R1);
            Eigen::Vector3d temp_t_1(_keyframes_show[i].second.second(0, 3), _keyframes_show[i].second.second(1, 3), _keyframes_show[i].second.second(2, 3));

            Eigen::Matrix3d R_constraint;
            Eigen::Vector3d t_constraint;

            R_constraint = temp_q_2.toRotationMatrix().inverse() * temp_q_1.toRotationMatrix();
            t_constraint = temp_q_2.toRotationMatrix().inverse() * (temp_t_1 - temp_t_2);

            Eigen::Quaterniond q_constraint(R_constraint);
            Sophus::SE3 se3_1(temp_q_1, temp_t_1);
            Sophus::SE3 se3_2(temp_q_2, temp_t_2);
            Sophus::SE3 se3_constraint(q_constraint, t_constraint);
            se3_constraint = se3_2.inverse() * se3_1;
            // Sophus::SE3 SE3_constraint(q_constraint, t_constraint);
            EdgeRelativeMotion *e // new edge with correct cohort for caching
                = new EdgeRelativeMotion();
            VertexPose *vp0 =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i)->second);
            VertexPose *vp1 =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i + 1)->second);
            e->setVertex(0, vp0); // first viewpoint
            e->setVertex(1, vp1); // second viewpoint
            e->setMeasurement(se3_constraint);
            e->information() = 100 * Eigen::Matrix<double, 6, 6>::Identity();

            // use this for point-point
            //    e->information().setIdentity();

            // e->setRobustKernel(true);
            // e->setHuberWidth(0.01);

            optimizer.addEdge(e);
        }
        Eigen::Matrix3d loop_R2 = pose2.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_2(loop_R2);
        Eigen::Vector3d loop_t_2(pose2(0, 3), pose2(1, 3), pose2(2, 3));
        Eigen::Matrix3d loop_R1 = pose1.block<3, 3>(0, 0);
        Eigen::Quaterniond loop_q_1(loop_R1);
        Eigen::Vector3d loop_t_1(pose1(0, 3), pose1(1, 3), pose1(2, 3));
        Sophus::SE3 se3_loop1(loop_q_1, loop_t_1);
        Sophus::SE3 se3_loop2(loop_q_2, loop_t_2);
        Sophus::SE3 se3_loop = se3_loop2.inverse() * se3_loop1;
        EdgeRelativeMotion *e // new edge with correct cohort for caching
            = new EdgeRelativeMotion();
        VertexPose *vp0 =
            dynamic_cast<VertexPose *>(optimizer.vertices().find(index1)->second);
        VertexPose *vp1 =
            dynamic_cast<VertexPose *>(optimizer.vertices().find(index2)->second);
        vp0->setFixed(true);
        e->setMeasurement(se3_loop);
        e->setVertex(0, vp0); // first viewpoint
        e->setVertex(1, vp1); // second viewpoint
        e->information() = 1000 * Eigen::Matrix<double, 6, 6>::Identity();
        optimizer.addEdge(e);
        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();

        optimizer.setVerbose(true);

        optimizer.optimize(25);
        for (int i = 0; i < _keyframes_show.size(); i++)
        {
            VertexPose *vc =
                dynamic_cast<VertexPose *>(optimizer.vertices().find(i)->second);
            Sophus::SE3 pose = vc->estimate();

            _keyframes_show[i].second.second.block<3, 3>(0, 0) = pose.unit_quaternion().toRotationMatrix();
            _keyframes_show[i].second.second.block<3, 1>(0, 3) = pose.translation();
        }
    }
}
int Mapping::DetectLoopClosure(const Eigen::Matrix4d &pose, double &time_stamp)
{
    std::cout << "_keyframes.size():" << _keyframes.size() << std::endl;
    for (int i = 0; i < _keyframes.size(); i++)
    {
        Eigen::Matrix4d T_keyframe = _keyframes[i].second.second;
        double distance = calculateDistance(pose, T_keyframe);
        if (distance < _distance_threshold * 0.5 && std::abs(time_stamp * 1e-6 - _keyframes[i].second.first * 1e-6) > 20)
        {
            std::cout << "pose" << pose << std::endl;
            std::cout << "T_keyframe" << T_keyframe << std::endl;
            std::cout << "distance:" << distance << std::endl;
            std::cout << "LoopClosure" << std::endl;
            return i;
        }
    }
    return -1;
}
bool Mapping::judgeFeaturesDistribution(mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud)
{
    mypcl::PointCloud<mypcl::PointXYZI>::Ptr cloud_judge(new mypcl::PointCloud<mypcl::PointXYZI>);
    mypcl::copyPointCloud(*cloud, *cloud_judge);
    Eigen::Matrix<double, 4, -1> neighbors(4, cloud_judge->points.size());
    for (int i = 0; i < cloud_judge->points.size(); i++)
    {
        Eigen::Vector4d p;
        p[0] = cloud_judge->points[i].x;
        p[1] = cloud_judge->points[i].y;
        p[2] = cloud_judge->points[i].z;
        p[3] = 1;
        neighbors.col(i) = p;
    }
    neighbors.colwise() -= neighbors.rowwise().mean().eval();
    Eigen::Matrix4d cov = neighbors * neighbors.transpose() / cloud_judge->points.size();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d values;
    values = svd.singularValues();
    std::cout << "values:" << values << std::endl;
    if (values[0] / values[1] < 100)
    {
        return true;
    }
    else
    {
        return false;
    }
}
