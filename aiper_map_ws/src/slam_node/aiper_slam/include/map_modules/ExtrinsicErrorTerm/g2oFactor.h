#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
/**
 * 旋转在前的SO3+t类型pose，6自由度，存储时伪装为g2o::VertexSE3，供g2o_viewer查看
 */
class VertexPose : public g2o::BaseVertex<6, SE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose() {}

  bool read(std::istream& is) override {
		return true;
  }

  bool write(std::ostream& os) const override {
    return true;
  }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update_) {
    _estimate.so3() = _estimate.so3() * SO3::exp(Eigen::Map<const Vec3d>(&update_[0]));  // 旋转部分
    _estimate.translation() += Eigen::Map<const Vec3d>(&update_[3]);                     // 平移部分
    updateCache();
  }
};
/**
 * 6 自由度相对运动
 * 误差的平移在前，角度在后
 * 观测：T12
 */
class EdgeRelativeMotion : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeRelativeMotion(){}

  virtual void computeError() override {
    VertexPose* v1 = (VertexPose*)_vertices[0];
    VertexPose* v2 = (VertexPose*)_vertices[1];
    _error = (_measurement.inverse() * v2->estimate().inverse() * v1->estimate()).log();
  };

  virtual bool read(std::istream& is) override {
    return true;
  }

  virtual bool write(std::ostream& os) const override {
    return true;
  }

   private:
};