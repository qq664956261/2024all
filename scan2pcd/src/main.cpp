
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
struct Pose
{
    double x, y, yaw;

    Pose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
};

// 将角度从度转换为弧度
double degreesToRadians(double degrees)
{
    if (degrees <= -360.0)
    {
        degrees = degrees + 360.0;
    }
    return degrees * M_PI / 180.0;
}

// 根据当前位置、步长和偏航角更新位置
Pose updatePose(Pose currentPose, double step, double yawChangeDegrees)
{
    double yawRadians = degreesToRadians(currentPose.yaw + yawChangeDegrees);
    // std::cout << "yawRadians: " << yawRadians << std::endl;
    return {currentPose.x + step * cos(yawRadians), currentPose.y + step * sin(yawRadians), currentPose.yaw + yawChangeDegrees};
}

// 生成矩形轨迹的函数，考虑偏航角变化
std::vector<Pose> generateRectangularTrajectory(double length, double width, double pointsPerEdge)
{
    std::vector<Pose> trajectory;
    Pose currentPose(0, 2, 90); // 初始位置在（0,2)，偏航角0度指向右侧
    trajectory.push_back(currentPose);

    // 向下移动（调整偏航角为270度）
    for (int i = 0; i < pointsPerEdge; i++)
    {

        currentPose = updatePose(currentPose, 3.0 / pointsPerEdge, 0);
        std::cout << "i:" << i << std::endl;
        std::cout << "currentPose: " << currentPose.x << " " << currentPose.y << " " << currentPose.yaw << std::endl;
        trajectory.push_back(currentPose);
    }

    // 向右移动（偏航角0度）
    for (int i = 0; i < pointsPerEdge; i++)
    {
   
        currentPose = updatePose(currentPose, length / pointsPerEdge, i == 0 ? -90 : 0);
        std::cout << "i:" << i << std::endl;
        std::cout << "currentPose: " << currentPose.x << " " << currentPose.y << " " << currentPose.yaw << std::endl;
        trajectory.push_back(currentPose);
    }

    // 向上移动（偏航角90度）
    for (int i = 0; i < pointsPerEdge; i++)
    {

        currentPose = updatePose(currentPose, width / pointsPerEdge, i == 0 ? -90 : 0);
        std::cout << "i:" << i << std::endl;
        std::cout << "currentPose: " << currentPose.x << " " << currentPose.y << " " << currentPose.yaw << std::endl;
        trajectory.push_back(currentPose);
    }

    // 向左移动回到起始点（偏航角180度）
    for (int i = 0; i <= pointsPerEdge; i++)
    {

        currentPose = updatePose(currentPose, length / pointsPerEdge, i == 0 ? -90 : 0);
        std::cout << "i:" << i << std::endl;
        std::cout << "currentPose: " << currentPose.x << " " << currentPose.y << " " << currentPose.yaw << std::endl;
        trajectory.push_back(currentPose);
    }

    for (int i = 0; i <= pointsPerEdge; ++i)
    {
        trajectory.push_back(currentPose);
        //std::cout << "currentPose: " << currentPose.x << " " << currentPose.y << " " << currentPose.yaw << std::endl;
        currentPose = updatePose(currentPose, width / pointsPerEdge, i == 0 ? -88 : 0);
    }

    for (int i = 0; i <= pointsPerEdge; ++i)
    {
        trajectory.push_back(currentPose);
        currentPose = updatePose(currentPose, length / pointsPerEdge, i == 0 ? -90 : 0);
    }

    return trajectory;
}
void callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        // 判断每个点的距离是否越界
        if (msg->ranges[i] < 0.1 || msg->ranges[i] > 30)
            continue;

        // 获取第i个点与车辆自身的角度
        double real_angle = msg->angle_min + i * msg->angle_increment;

        // 将雷达的距离和角度转换为笛卡尔坐标
        double x = msg->ranges[i] * std::cos(real_angle);
        double y = msg->ranges[i] * std::sin(real_angle);
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = 0;
        p.intensity = 0;
        cloud->points.push_back(p);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    // ply保存
    pcl::io::savePLYFile("scan.ply", *cloud);
}
int num = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
void pointcloud2_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    *cloud_all += *cloud;
    num++;
    if (num > 20000)
    {
        cloud_all->width = cloud_all->points.size();
        cloud_all->height = 1;
        cloud_all->is_dense = false;
        pcl::io::savePLYFile("pointcloud_all111.ply", *cloud_all);
        num = 0;
    }
}

int main(int argc, char **argv)
{
    std::vector<Pose> trajectory = generateRectangularTrajectory(10.0, 5.0, 5000.0);
    std::ofstream odom("ArmyOdom.txt");
    std::ofstream ultra("ArmyUltra.txt");
    double time = 1679943859964383;
    for (const Pose &pose : trajectory)
    {

        double yaw;
        if (pose.yaw <= -360.0)
        {
            yaw = pose.yaw + 360.0;
        }
        yaw = yaw / 180.0 * 3.14;
        // std::cout << "Pose: (x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.yaw << ")" << yaw << std::endl;
        odom << std::to_string(time) << " " << std::to_string(pose.x) << " " << std::to_string(pose.y) << " " << std::to_string(0) << " " << std::to_string(0) << " " << std::to_string(0) << " " << std::to_string(yaw) << std::endl;

        double fov_rad = 15;
        double sonar_base_x = 0.156;
        double sonar_base_y = 0.148;
        double sonar_base_yaw = 1.57;
        double length;

        Eigen::Matrix4d T_wc = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond cur_Q_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
        cur_Q_.normalize();
        T_wc.block<3, 3>(0, 0) = cur_Q_.toRotationMatrix();
        T_wc.block<3, 1>(0, 3) = Eigen::Vector3d(pose.x, pose.y, 0);
        Eigen::Vector4d p_w = Eigen::Vector4d(2 * pose.x, 2 * pose.y, 0, 1);
        // std::cout<<"p_w:"<<p_w<<std::endl;
        Eigen::Vector4d p_base = T_wc.inverse() * p_w;
        // std::cout<<"p_base:"<<p_base<<std::endl;
        Eigen::Matrix3d R_sonar = Eigen::Matrix3d::Identity();
        R_sonar << cos(sonar_base_yaw), -sin(sonar_base_yaw), 0,
            sin(sonar_base_yaw), cos(sonar_base_yaw), 0,
            0, 0, 1;
        Eigen::Vector3d p_sonar = Eigen::Vector3d(sonar_base_x, sonar_base_y, 0);
        Eigen::Matrix4d T_sonar = Eigen::Matrix4d::Identity();
        T_sonar.block<3, 3>(0, 0) = R_sonar;
        T_sonar.block<3, 1>(0, 3) = p_sonar;
        Eigen::Vector4d p_sonar_ = T_sonar.inverse() * p_base;

        ultra << std::to_string(time) << " " << std::to_string(0.368) << " " << std::to_string(0.322) << std::endl;
        time += 10000;
    }
    ros::init(argc, argv, "scan2pcd");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/front_scan");
    ros::Subscriber sub = nh.subscribe(cloud_topic, 10, callback);
    std::string pointcloud2_topic;
    nh.param<std::string>("pointcloud2_topic", pointcloud2_topic, "/scan_matched_points2");
    ros::Subscriber sub2 = nh.subscribe(pointcloud2_topic, 10, pointcloud2_callback);
    // 开始循环等待回调函数
    ros::spin();

    return 0;
}
