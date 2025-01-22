#include "pointType.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stack>
#include <Eigen/Dense>
using namespace std;
using Point = pair<double, double>;

void BinToPly(const std::string& bin_file_path, std::shared_ptr<mypcl::PointCloud<mypcl::PointXYZI>> cloud) {
        // 读取二进制文件中的点
    std::ifstream bin_file(bin_file_path, std::ios::binary);
    if (!bin_file) {
        std::cerr << "Failed to open binary file: " << bin_file_path << std::endl;
        return;
    }
    float x, y, z;
    while (bin_file.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
           bin_file.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
           bin_file.read(reinterpret_cast<char*>(&z), sizeof(float))) {
        mypcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->points.push_back(point);
    }


}

// 计算两点间的直线参数（a, b），直线方程为 y = ax + b
bool calculateLineModel(const Point &p1, const Point &p2, double &a, double &b)
{
    if (fabs(p1.first - p2.first) < 1e-6)
    {
        return false; // 避免除以零
    }
    a = (p2.second - p1.second) / (p2.first - p1.first);
    b = p1.second - a * p1.first;
    return true;
}

// 计算点到直线 y = ax + b 的距离
double pointToLineDistance(const Point &point, double a, double b)
{
    return fabs(a * point.first - point.second + b) / sqrt(a * a + 1);
}

// 使用 RANSAC 算法拟合直线
bool ransacLineFitting(const vector<Point> &points, double distanceThreshold, double inlierRatio, double &bestA, double &bestB)
{
    int maxInliers = 0;
    int iterations = 100;
    srand(static_cast<unsigned int>(time(0))); // 设置随机种子

    for (int i = 0; i < iterations; ++i)
    {
        // 随机选择两个点
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();
        if (idx1 == idx2)
            continue;

        double a, b;
        if (!calculateLineModel(points[idx1], points[idx2], a, b))
        {
            continue;
        }

        // 计算内点数
        int inliers = 0;
        for (const auto &point : points)
        {
            if (pointToLineDistance(point, a, b) < distanceThreshold)
            {
                ++inliers;
            }
        }

        // 更新最佳直线模型
        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            bestA = a;
            bestB = b;
        }

        // 如果内点比例达到预设值，则提前退出
        if (static_cast<double>(maxInliers) / points.size() >= inlierRatio)
        {
            return true;
        }
    }

    return maxInliers > 0;
}

// 剔除属于直线的内点
void removeInliers(vector<Point> &points, double a, double b, double distanceThreshold)
{
    vector<Point> remainingPoints;
    for (const auto &point : points)
    {
        if (pointToLineDistance(point, a, b) >= distanceThreshold)
        {
            remainingPoints.push_back(point);
        }
    }
    points = remainingPoints;
}

// 主函数：在点云中重复拟合并剔除直线
void iterativeLineFitting(vector<Point> &points, double distanceThreshold, double inlierRatio, int minPoints, int &lineNum)
{
    while (points.size() >= minPoints)
    {
        double bestA = 0, bestB = 0;
        if (ransacLineFitting(points, distanceThreshold, inlierRatio, bestA, bestB))
        {
            //cout << "Fitted line: y = " << bestA << "x + " << bestB << endl;
            removeInliers(points, bestA, bestB, distanceThreshold);
            //cout << "Remaining points: " << points.size() << endl;
            lineNum++;
        }
        else
        {
            break; // 无法找到新的直线
        }
    }
}

// 计算两点之间的欧几里得距离
double euclideanDistance(const Point &p1, const Point &p2)
{
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

// 计算点到线段的最短距离
double pointToSegmentDistance(const Point &point, const Point &segStart, const Point &segEnd)
{
    double x0 = point.first, y0 = point.second;
    double x1 = segStart.first, y1 = segStart.second;
    double x2 = segEnd.first, y2 = segEnd.second;

    double dx = x2 - x1;
    double dy = y2 - y1;
    if (dx == 0 && dy == 0)
    {
        // segStart == segEnd 的情况，线段退化为一个点
        return euclideanDistance(point, segStart);
    }

    // 投影的比例因子
    double t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy);
    if (t < 0)
    {
        // 离segStart更近
        return euclideanDistance(point, segStart);
    }
    else if (t > 1)
    {
        // 离segEnd更近
        return euclideanDistance(point, segEnd);
    }

    // 投影点在线段上
    double projX = x1 + t * dx;
    double projY = y1 + t * dy;
    return euclideanDistance(point, {projX, projY});
}

// 计算每个凸包点到最小边界框的最小距离
vector<double> calculateDistancesToBoundingBox(const vector<Point> &hull, const vector<Point> &box)
{
    vector<double> distances;

    for (const auto &point : hull)
    {
        double minDistance = numeric_limits<double>::max();

        // 遍历边界框的每条边，计算点到边的最小距离
        for (size_t i = 0; i < box.size(); ++i)
        {
            double x_dis = std::fabs(point.first - box[i].first);
            double y_dis = std::fabs(point.second - box[i].second);
            // std::cout << "point.first:" << point.first << std::endl;
            // std::cout << "point.second:" << point.second << std::endl;
            // std::cout << " box[i].first:" <<  box[i].first << std::endl;
            // std::cout << "box[i].second:" << box[i].second << std::endl;
            // std::cout << "x_dis:" << x_dis << std::endl;
            // std::cout << "y_dis:" << y_dis << std::endl;
            double distance = 0;
            if (x_dis < y_dis)
            {
                distance = x_dis;
            }
            else
            {
                distance = y_dis;
            }
            minDistance = min(minDistance, distance);
        }

        distances.push_back(minDistance);
        //std::cout<<"id:"<<distances.size() - 1<<std::endl;
    }

    return distances;
}
// 计算点积
double crossProduct(const pair<double, double> &p1, const pair<double, double> &p2, const pair<double, double> &p3)
{
    return (p2.first - p1.first) * (p3.second - p1.second) - (p2.second - p1.second) * (p3.first - p1.first);
}
// 计算凸包的面积
double convexHullArea(const vector<pair<double, double>> &points, vector<pair<double, double>> &hulls)
{
    int n = points.size();
    if (n < 3)
        return 0; // Convex hull can't be formed with less than 3 points.

    vector<pair<double, double>> hull;

    // 将点按x坐标排序
    vector<pair<double, double>> sortedPoints = points;
    sort(sortedPoints.begin(), sortedPoints.end());

    // 计算凸包的下半部分
    for (auto &p : sortedPoints)
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    // 计算凸包的上半部分
    size_t t = hull.size() + 1;
    for (int i = sortedPoints.size() - 2; i >= 0; --i)
    {
        while (hull.size() >= t && crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], sortedPoints[i]) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(sortedPoints[i]);
    }
    auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    for (int i = 0; i < hull.size(); i++)
    {
        mypcl::PointXYZI point;
        point.x = hull[i].first;
        point.y = hull[i].second;
        point.z = 0;
        cloud->points.push_back(point);
    }
    if (cloud->points.size() != 0)
        mypcl::savePLYFileBinary("cloud.ply", *cloud);
    hulls = hull;

    // 计算凸包的面积
    double area = 0;
    for (int i = 0; i < hull.size() - 1; ++i)
    {
        area += hull[i].first * hull[i + 1].second - hull[i + 1].first * hull[i].second;
    }
    area = fabs(area) / 2.0;
    return area;
}

// 计算点云的最小边界框面积
double boundingBoxArea(const vector<pair<double, double>> &points, vector<pair<double, double>> &boxs)
{
    double minX = points[0].first, maxX = points[0].first;
    double minY = points[0].second, maxY = points[0].second;

    for (auto &p : points)
    {
        minX = min(minX, p.first);
        maxX = max(maxX, p.first);
        minY = min(minY, p.second);
        maxY = max(maxY, p.second);
    }
    boxs.push_back(make_pair(minX, maxY));
    boxs.push_back(make_pair(minX, minY));
    boxs.push_back(make_pair(maxX, maxY));
    boxs.push_back(make_pair(maxX, minY));

    double width = maxX - minX;
    double height = maxY - minY;
    return width * height;
}

// 判断点云是否接近矩形
bool isRectangle( vector<pair<double, double>> &points)
{
    vector<pair<double, double>> hull;
    vector<pair<double, double>> box;
    double bbArea = boundingBoxArea(points, box);
    double hullArea = convexHullArea(points, hull);

    vector<double> distances = calculateDistancesToBoundingBox(hull, box);
    double dis = 0;
    // 输出每个凸包点到边界框的最小距离
    for (size_t i = 0; i < distances.size(); ++i)
    {
        cout << "Distance from hull point " << i << " to bounding box: " << distances[i] << endl;
        dis += distances[i];
    }
    dis = dis / distances.size();
    std::cout << "dis:" << dis << std::endl;

    // 计算两者的比值
    double ratio = hullArea / bbArea;

    // 如果比值接近 1，则认为是矩形（这里使用一个阈值判断）
    std::cout << "ratio:" << ratio << std::endl;
    //return abs(ratio - 1.0) < 0.1;

    double distanceThreshold = 0.025; // 内点距离阈值
    double inlierRatio = 0.8;        // 内点比例阈值
    int minPoints = points.size() * 0.1;               // 最少剩余点数
    int lineNum = 0;

    iterativeLineFitting(points, distanceThreshold, inlierRatio, minPoints, lineNum);
    std::cout << "lineNum:" << lineNum << std::endl;
    return (dis < 0.15 && lineNum < 10);
}

// 计算点云的凸包（二维情况）
vector<pair<double, double>> convexHull(const vector<pair<double, double>> &points)
{
    int n = points.size();
    if (n < 3)
        return points; // 不能形成凸包时返回点本身

    vector<pair<double, double>> sortedPoints = points;
    sort(sortedPoints.begin(), sortedPoints.end());

    vector<pair<double, double>> hull;

    // 计算凸包的下半部分
    for (auto &p : sortedPoints)
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    // 计算凸包的上半部分
    size_t t = hull.size() + 1;
    for (int i = sortedPoints.size() - 2; i >= 0; --i)
    {
        while (hull.size() >= t && crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], sortedPoints[i]) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(sortedPoints[i]);
    }

    hull.pop_back(); // 最后一个点重复，去掉
    return hull;
}

// 计算向量的点积
double dotProduct(const pair<double, double> &A, const pair<double, double> &B)
{
    return A.first * B.first + A.second * B.second;
}

// 计算向量的模长
double magnitude(const pair<double, double> &v)
{
    return sqrt(v.first * v.first + v.second * v.second);
}

// 计算两向量之间的夹角（以度数为单位）
double angleBetweenVectors(const pair<double, double> &A, const pair<double, double> &B)
{
    double dot = dotProduct(A, B);
    double magA = magnitude(A);
    double magB = magnitude(B);
    double cosTheta = dot / (magA * magB);
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta)); // 防止浮点误差
    double theta = acos(cosTheta);                      // 夹角弧度
    return theta * 180.0 / M_PI;                        // 转换为度数
}


// 计算多边形的面积（适用于凹包或凸包）
double polygonArea(const vector<pair<double, double>> &polygon)
{
    double area = 0;
    int n = polygon.size();
    for (int i = 0; i < n; ++i)
    {
        int j = (i + 1) % n;
        area += polygon[i].first * polygon[j].second - polygon[j].first * polygon[i].second;
    }
    return fabs(area) / 2.0;
}


Eigen::Matrix4d mainDirection(std::shared_ptr<mypcl::PointCloud<mypcl::PointXYZI>> cloud)
{
        Eigen::Matrix<double, 4, -1> neighbors(4, cloud->points.size());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector4d p;
        p[0] = cloud->points[i].x;
        p[1] = cloud->points[i].y;
        p[2] = cloud->points[i].z;
        p[3] = 1;
        neighbors.col(i) = p;
    }
    neighbors.colwise() -= neighbors.rowwise().mean().eval();
    Eigen::Matrix4d cov = neighbors * neighbors.transpose() / cloud->points.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov.block<3, 3>(0, 0));
    Eigen::Vector3d values;
    values = eigensolver.eigenvalues();
    std::cout << "values:" << values << std::endl;
    Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
    Eigen::Vector3d maxEigenVector = eigenvectors.col(0); // 最大特征值对应的特征向量
    Eigen::Vector3d midEigenVector = eigenvectors.col(1); // 中间特征值对应的特征向量
    std::cout << "maxEigenVector:" << maxEigenVector << std::endl;
    std::cout << "midEigenVector:" << midEigenVector << std::endl;
    std::cout << "minEigenVector:" << eigenvectors.col(2) << std::endl;


    // 构造旋转矩阵，将特征向量对齐到 x、y、z 轴
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << eigenvectors.col(2), eigenvectors.col(1), eigenvectors.col(0);
    double theta = std::asin(-eigenvectors.col(1)[0]);
    std::cout<<"theta:"<<theta<<std::endl;
    rotationMatrix << std::cos(theta) , -std::sin(theta), 0 , std::sin(theta), std::cos(theta), 0, 0, 0, 1;
    std::cout<<"rotationMatrix before:"<<rotationMatrix<<std::endl;
    // Eigen::Quaterniond q (rotationMatrix);
    // std::cout<<"q.w():"<<q.w()<<std::endl;
    // std::cout<<"q.x():"<<q.x()<<std::endl;
    // std::cout<<"q.y():"<<q.y()<<std::endl;
    // std::cout<<"q.z():"<<q.z()<<std::endl;
    // q.normalize();
    // std::cout<<"q.w():"<<q.w()<<std::endl;
    // std::cout<<"q.x():"<<q.x()<<std::endl;
    // std::cout<<"q.y():"<<q.y()<<std::endl;
    // std::cout<<"q.z():"<<q.z()<<std::endl;
    // rotationMatrix = q.toRotationMatrix();
    std::cout<<"rotationMatrix after:"<<rotationMatrix<<std::endl;


    Eigen::Matrix<double, 4, -1> neighbors1(4, cloud->points.size());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector4d p;
        p[0] = cloud->points[i].x;
        p[1] = cloud->points[i].y;
        p[2] = cloud->points[i].z;
        p[3] = 1;
        neighbors1.col(i) = p;
    }
    neighbors1.colwise() -= neighbors1.rowwise().mean().eval();
    Eigen::Matrix4d cov1 = neighbors1 * neighbors1.transpose() / cloud->points.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver1(cov1.block<3, 3>(0, 0));
    Eigen::Vector3d values1;
    values1 = eigensolver1.eigenvalues();
    std::cout << "values:" << values1 << std::endl;
    Eigen::Matrix3d eigenvectors1 = eigensolver1.eigenvectors();
    Eigen::Vector3d maxEigenVector1 = eigenvectors1.col(0); // 最大特征值对应的特征向量
    Eigen::Vector3d midEigenVector1 = eigenvectors1.col(1); // 中间特征值对应的特征向量
    std::cout << "maxEigenVector:" << maxEigenVector1 << std::endl;
    std::cout << "midEigenVector:" << midEigenVector1 << std::endl;
    std::cout << "minEigenVector:" << eigenvectors1.col(2) << std::endl;

    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<2,2>(0,0) = rotationMatrix.block<2,2>(0,0);
    return T;

}

int main()
{
    auto cloud = std::make_shared<mypcl::PointCloud<mypcl::PointXYZI>>();
    //mypcl::loadPLYFile("/home/zc/1106/maps/map/map.ply", *cloud);
    //mypcl::loadPLYFile("/home/zc/1106/map1010.ply", *cloud);
    BinToPly("/home/zc/1106/point_cloud1.bin", cloud);
    std::cout << "cloud->points.size():" << cloud->points.size() << std::endl;
    mypcl::voxelGridFilter(*cloud, *cloud, 1);
    std::cout << "cloud->points.size():" << cloud->points.size() << std::endl;


    Eigen::Matrix4d T = mainDirection(cloud);
    mypcl::transformPointCloud(*cloud, *cloud, T.cast<float>());
    std::cout<<"rotationMatrix:"<<T<<std::endl;
    mypcl::savePLYFileBinary("cloud_rot.ply", *cloud);


    vector<pair<double, double>> points;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        pair<double, double> point;
        point.first = cloud->points[i].x;
        point.second = cloud->points[i].y;
        points.push_back(point);
    }
    if (isRectangle(points))
    {
        cout << "The point cloud is likely a rectangle." << endl;
    }
    else
    {
        cout << "The point cloud is not a rectangle." << endl;
    }


    return 0;
}