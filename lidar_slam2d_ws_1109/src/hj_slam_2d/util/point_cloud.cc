#include "point_cloud.h"

namespace hjSlam_2d
{
    PointCloud::PointCloud() {}
    PointCloud::PointCloud(std::vector<PointCloud::PointType> points)
        : points_(std::move(points)) {}
    PointCloud::PointCloud(std::vector<PointType> points,
                           std::vector<float> intensities)
        : points_(std::move(points)), intensities_(std::move(intensities))
    {
    }

    size_t PointCloud::size() const { return points_.size(); }
    bool PointCloud::empty() const { return points_.empty(); }

    const std::vector<PointCloud::PointType> &PointCloud::points() const
    {
        return points_;
    }
    const std::vector<float> &PointCloud::intensities() const
    {
        return intensities_;
    }
    const PointCloud::PointType &PointCloud::operator[](const size_t index) const
    {
        return points_[index];
    }

    PointCloud::ConstIterator PointCloud::begin() const { return points_.begin(); }
    PointCloud::ConstIterator PointCloud::end() const { return points_.end(); }

    void PointCloud::push_back(PointCloud::PointType value)
    {
        points_.push_back(std::move(value));
    }

    PointCloud TransformPointCloud(const PointCloud &point_cloud,
                                   const transform::Rigid3f &transform)
    {
        std::vector<RangefinderPoint> points;
        points.reserve(point_cloud.size());
        for (const RangefinderPoint &point : point_cloud.points())
        {
            points.emplace_back(transform * point);
        }
        return PointCloud(points, point_cloud.intensities());
    }

    TimedPointCloud TransformTimedPointCloud(const TimedPointCloud &point_cloud,
                                             const transform::Rigid3f &transform)
    {
        TimedPointCloud result;
        result.reserve(point_cloud.size());
        for (const TimedRangefinderPoint &point : point_cloud)
        {
            result.push_back(transform * point);
        }
        return result;
    }

    RangeData TransformRangeData(const RangeData &range_data,
                                 const transform::Rigid3f &transform)
    {
        return RangeData{
            transform * range_data.origin,
            TransformPointCloud(range_data.returns, transform),
            TransformPointCloud(range_data.misses, transform),
        };
    }

}