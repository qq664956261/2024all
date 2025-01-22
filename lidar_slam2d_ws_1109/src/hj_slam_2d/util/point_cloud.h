#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <vector>
#include <Eigen/Core>
#include "time.h"
#include "rigid_transform.h"

namespace hjSlam_2d
{

    // Stores 3D position of a point observed by a rangefinder sensor.
  struct RangefinderPoint
  {
    Eigen::Vector3f position;
  };

  class PointCloud
  {
  public:
    using PointType = RangefinderPoint;

    PointCloud();
    explicit PointCloud(std::vector<PointType> points);
    PointCloud(std::vector<PointType> points, std::vector<float> intensities);

    // Returns the number of points in the point cloud.
    size_t size() const;
    // Checks whether there are any points in the point cloud.
    bool empty() const;

    const std::vector<PointType> &points() const;
    const std::vector<float> &intensities() const;
    const PointType &operator[](const size_t index) const;

    // Iterator over the points in the point cloud.
    using ConstIterator = std::vector<PointType>::const_iterator;
    ConstIterator begin() const;
    ConstIterator end() const;

    void push_back(PointType value);

    // Creates a PointCloud consisting of all the points for which `predicate`
    // returns true, together with the corresponding intensities.
    template <class UnaryPredicate>
    PointCloud copy_if(UnaryPredicate predicate) const
    {
      std::vector<PointType> points;
      std::vector<float> intensities;

      // Note: benchmarks show that it is better to have this conditional outside
      // the loop.
      if (intensities_.empty())
      {
        for (size_t index = 0; index < size(); ++index)
        {
          const PointType &point = points_[index];
          if (predicate(point))
          {
            points.push_back(point);
          }
        }
      }
      else
      {
        for (size_t index = 0; index < size(); ++index)
        {
          const PointType &point = points_[index];
          if (predicate(point))
          {
            points.push_back(point);
            intensities.push_back(intensities_[index]);
          }
        }
      }
      return PointCloud(points, intensities);
    }

  private:
    // For 2D points, the third entry is 0.f.
    std::vector<PointType> points_;
    // Intensities are optional. If non-empty, they must have the same size as
    // points.
    std::vector<float> intensities_;
  };

  template <class T>
  inline RangefinderPoint operator*(const transform::Rigid3<T> &lhs,
                                    const RangefinderPoint &rhs)
  {
    RangefinderPoint result = rhs;
    result.position = lhs * rhs.position;
    return result;
  }

  struct TimedRangefinderPoint
  {
    Eigen::Vector3f position;
    float time;
  };

  template <class T>
  inline TimedRangefinderPoint operator*(const transform::Rigid3<T> &lhs,
                                         const TimedRangefinderPoint &rhs)
  {
    TimedRangefinderPoint result = rhs;
    result.position = lhs * rhs.position;
    return result;
  }

  inline bool operator==(const RangefinderPoint &lhs,
                         const RangefinderPoint &rhs)
  {
    return lhs.position == rhs.position;
  }

  inline bool operator==(const TimedRangefinderPoint &lhs,
                         const TimedRangefinderPoint &rhs)
  {
    return lhs.position == rhs.position && lhs.time == rhs.time;
  }

  using TimedPointCloud = std::vector<TimedRangefinderPoint>;

  struct PointCloudWithIntensities
  {
    hjSlam_2d::common::Time time; // 点云最后一个点的时间
    Eigen::Vector3f origin;        // 以tracking_frame_到雷达坐标系的坐标变换为原点
    TimedPointCloud points;
    std::vector<float> intensities;
  };

  struct RangeData
  {
    Eigen::Vector3f origin;
    PointCloud returns;
    PointCloud misses;
    //test
    PointCloud points_to_matched;
  };

  RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);
                             
  // Transforms 'point_cloud' according to 'transform'.
  PointCloud TransformPointCloud(const PointCloud &point_cloud,
                                 const transform::Rigid3f &transform);

  // Transforms 'point_cloud' according to 'transform'.
  TimedPointCloud TransformTimedPointCloud(const TimedPointCloud &point_cloud,
                                           const transform::Rigid3f &transform);

}

#endif