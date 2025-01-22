//create by zhangcheng 2024/05/11
#ifndef MYPOINTTYPE_H
#define MYPOINTTYPE_H
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
namespace mypcl
{
    struct Voxel
    {
        float x = 0, y = 0, z = 0;
        int count = 0;
        template <typename PointT>
        void addVoxelPoint(const PointT &point)
        {
            x += point.x;
            y += point.y;
            z += point.z;
            count++;
        }
        template <typename PointT>
        void getAveragePoint( PointT& point) const
        {
            point.x = x / count;
            point.y = y / count;
            point.z = z / count;
            point.intensity = 0;
        }
    };

    struct PointXYZI
    {
        float x, y, z;   // 3D坐标
        float intensity; // 强度值

        PointXYZI() : x(0.0f), y(0.0f), z(0.0f), intensity(0.0f) {}
        PointXYZI(float _x, float _y, float _z, float _intensity) : x(_x), y(_y), z(_z), intensity(_intensity) {}
    };

    template <typename PointT>
    class PointCloud
    {
    public:
        typedef std::shared_ptr<PointCloud<PointT>> Ptr;
        std::vector<PointT> points; // 存储点的容器

        PointCloud() {}

        void addPoint(const PointT &point)
        {
            points.push_back(point);
        }

        void reset(PointCloud<PointT>::Ptr newCloudPtr)
        {
            *this = *newCloudPtr;
        }
        // 实现点云累加
        PointCloud<PointT> &operator+=(const PointCloud<PointT> &other)
        {
            points.insert(points.end(), other.points.begin(), other.points.end());
            return *this; // 返回当前对象的引用，以支持链式调用
        }

        bool empty() const
        {
            return points.empty();
        }
        int size() const
        {
            return points.size();
        }
    };
    template <typename PointT>
    // 深拷贝函数
    void copyPointCloud(const PointCloud<PointT> &source, PointCloud<PointT> &destination)
    {
        // 确保目标点云是空的，或者清空目标点云
        destination.points.clear();

        // 遍历源点云中的所有点，并逐个复制
        for (const auto &point : source.points)
        {
            destination.points.push_back(point);
        }
    }
    template <typename PointT>
    // 手写ASCII PLY文件保存函数
    bool savePLYFileBinary(const std::string &filename, const PointCloud<PointT> &cloud)
    {
        std::ofstream ofs(filename);
        if (!ofs.is_open())
        {
            std::cerr << "Error opening file for writing: " << filename << std::endl;
            return false;
        }

        // 写入PLY文件头
        ofs << "ply\n"
            << "format ascii 1.0\n"
            << "element vertex " << cloud.points.size() << "\n"
            << "property float x\n"
            << "property float y\n"
            << "property float z\n"
            << "property float intensity\n"
            << "end_header\n";

        // 写入点数据
        for (const auto &point : cloud.points)
        {
            ofs << point.x << " " << point.y << " " << point.z << " " << point.intensity << "\n";
        }

        ofs.close();

        return true;
    }
    template <typename PointT>
    void transformPointCloud(const PointCloud<PointT> &source,
                             PointCloud<PointT> &target,
                             const Eigen::Matrix4f &T)
    {
        // 检查source和target是否指向相同的对象
        if (&source == &target)
        {
            // 缓存原始点云，以避免直接覆盖
            PointCloud<PointT> temp(source);
            // 清除目标点云中的点，用于存储变换后的点云
            target.points.clear();
            // 遍历复制的点云进行坐标变换，并将结果存储到目标点云
            for (const auto &point : temp.points)
            {
                Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
                Eigen::Vector4f p_transformed = T * p;
                PointT transformed_point;
                transformed_point.x = p_transformed(0);
                transformed_point.y = p_transformed(1);
                transformed_point.z = p_transformed(2);
                transformed_point.intensity = point.intensity;
                // ... 如果存在其他属性，则复制 ...
                target.addPoint(transformed_point);
            }
        }
        else
        {
            // 如果source和target不同，则直接进行变换
            target.points.clear(); // 清除目标点云中的点
            for (const auto &point : source.points)
            {
                Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
                Eigen::Vector4f p_transformed = T * p;
                PointT transformed_point;
                transformed_point.x = p_transformed(0);
                transformed_point.y = p_transformed(1);
                transformed_point.z = p_transformed(2);
                transformed_point.intensity = point.intensity;
                // ... 如果存在其他属性，则复制 ...
                target.addPoint(transformed_point);
            }
        }
    }

    template <typename PointT>
    bool loadPLYFile(const std::string &filename, PointCloud<PointT> &cloud)
    {
        std::ifstream ifs(filename);
        if (!ifs.is_open())
        {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return false;
        }

        std::string line;
        bool header_passed = false;
        while (std::getline(ifs, line))
        {
            std::istringstream iss(line);
            if (line == "end_header")
            {
                header_passed = true;
                continue;
            }

            if (!header_passed)
            {
                // 处理文件头部信息，这里暂时忽略
                continue;
            }

            // 解析点数据
            float x, y, z, intensity;
            if (!(iss >> x >> y >> z >> intensity))
                break; // 格式化失败或达到文件末尾

            cloud.addPoint({x, y, z, intensity});
        }

        return true;
    }

    template <typename PointT>
    void voxelGridFilter(const PointCloud<PointT> &input, PointCloud<PointT> &output, float voxelSize)
    {
        std::unordered_map<int64_t, Voxel> voxelMap;
        auto hash = [voxelSize](float x, float y, float z) -> int64_t
        {
            return static_cast<int64_t>(std::floor(x / voxelSize)) + static_cast<int64_t>(std::floor(y / voxelSize)) * 10007 + static_cast<int64_t>(std::floor(z / voxelSize)) * 100000007;
        };

        // 构建体素网格
        for (const auto &point : input.points)
        {
            int64_t idx = hash(point.x, point.y, point.z);
            voxelMap[idx].addVoxelPoint(point);
        }

        // 获取每个体素的平均点，并构建输出点云
        output.points.clear();
        for (const auto &voxel : voxelMap)
        {
            PointT point;
            voxel.second.getAveragePoint(point);
            output.addPoint(point);
        }
    }
}

#endif
