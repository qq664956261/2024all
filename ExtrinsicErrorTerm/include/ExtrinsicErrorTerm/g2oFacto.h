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
        // double data[7];
        // for (int i = 0; i < 7; i++) {
        //     is >> data[i];
        // }
        // setEstimate(SE3(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2])));
		return true;
    }

    bool write(std::ostream& os) const override {
        // os << "VERTEX_SE3:QUAT ";
        // os << id() << " ";
        // Quatd q = _estimate.unit_quaternion();
        // os << _estimate.translation().transpose() << " ";
        // os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << std::endl;
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
class EdgeRelativeMotion : public g2o::BaseBinaryEdge<6, SE3, VertexPose, VertexPose> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRelativeMotion(){}
    // EdgeRelativeMotion(VertexPose* v1, VertexPose* v2, const SE3& obs) {
    //     setVertex(0, v1);
    //     setVertex(1, v2);
    //     setMeasurement(obs);
    // }

    void computeError() override {
        VertexPose* v1 = (VertexPose*)_vertices[0];
        VertexPose* v2 = (VertexPose*)_vertices[1];
        SE3 T21 = v2->estimate().inverse() * v1->estimate();
        _error = (_measurement.inverse() * v2->estimate().inverse() * v1->estimate()).log();
    };

    virtual bool read(std::istream& is) override {
        // double data[7];
        // for (int i = 0; i < 7; i++) {
        //     is >> data[i];
        // }
        // Quatd q(data[6], data[3], data[4], data[5]);
        // q.normalize();
        // setMeasurement(SE3(q, Vec3d(data[0], data[1], data[2])));
        // for (int i = 0; i < information().rows() && is.good(); i++) {
        //     for (int j = i; j < information().cols() && is.good(); j++) {
        //         is >> information()(i, j);
        //         if (i != j) information()(j, i) = information()(i, j);
        //     }
        // }
        return true;
    }

    virtual bool write(std::ostream& os) const override {
        // os << "EDGE_SE3:QUAT ";
        // auto* v1 = static_cast<VertexPose*>(_vertices[0]);
        // auto* v2 = static_cast<VertexPose*>(_vertices[1]);
        // os << v1->id() << " " << v2->id() << " ";
        // SE3 m = _measurement;
        // Eigen::Quaterniond q = m.unit_quaternion();
        // os << m.translation().transpose() << " ";
        // os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // // information matrix
        // for (int i = 0; i < information().rows(); i++) {
        //     for (int j = i; j < information().cols(); j++) {
        //         os << information()(i, j) << " ";
        //     }
        // }
        // os << std::endl;
        return true;
    }

   private:
};