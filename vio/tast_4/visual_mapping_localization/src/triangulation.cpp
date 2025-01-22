#include <iostream>
#include "triangulation.h"

namespace VISUAL_MAPPING {
    void Triangulation::triangulate(Frame *frame1, Frame *frame2, std::vector<cv::DMatch> &matches)
    {

    }

    std::pair<double, double> Triangulation::triangulate(Camera *camera1, Camera *camera2,
                                      const Eigen::Vector2d &kp1, const Eigen::Vector2d &kp2,
                                      const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
    {
        // 将像素坐标反投影到相机坐标系的方法
        // 将2D关键点 kp 反投影为3D射线（单位向量），表示从相机中心指向该关键点的方向
        Eigen::Vector3d r1 = camera1->unproject(kp1);
        Eigen::Vector3d r2 = camera2->unproject(kp2);
    //    std::cout<<"r1 "<<r1<<std::endl;
    //    std::cout<<"r2 "<<r2<<std::endl;

        // 1. Check parallax
        // R 是从相机2到相机1的旋转矩阵，表示相机2的坐标系如何与相机1的坐标系对齐。
        // r21 是将r2（相机2中的射线）旋转到相机1的坐标系下的结果。
        Eigen::Vector3d r21= R * r2;
        // r1.dot(r21) 是向量 r1 和 r21 的点积（内积）。
        // r1.norm() 和 r21.norm() 分别是向量 r1 和 r21 的模（即长度）。由于 r1 和 r21 是单位向量，它们的模等于1。
        const double cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());
        // 当 cos(θ) 接近1时，表示 θ 接近0度，即两条射线几乎是平行的。
        //std::cout<<"cosParallaxRays:"<<cosParallaxRays<<std::endl;
        if (cosParallaxRays > 0.9998) {
            return std::make_pair(-1, 100);
        }

        // 2. try to triangulate
        // TODO : 采用SVD求解三角化问题
        Eigen::Vector3d x3D;
         Eigen::Matrix3d R21 = R.transpose();
        Eigen::Vector3d t21 = -R21 * t;

        Eigen::Matrix4d A;
        Eigen::Matrix<double,3,4> Tcw1;
        Eigen::Matrix<double,3,4> Tcw2;
        Tcw1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero();
        Tcw2 << R21, t21;
        A.row(0) = r1(0) * Tcw1.row(2) - Tcw1.row(0);
        A.row(1) = r1(1) * Tcw1.row(2) - Tcw1.row(1);
        A.row(2) = r2(0) * Tcw2.row(2) - Tcw2.row(0);
        A.row(3) = r2(1) * Tcw2.row(2) - Tcw2.row(1);
//        std::cout<<" A "<<A<<std::endl;
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4d x3D_homogeneous = svd.matrixV().col(3);
        x3D = x3D_homogeneous.head(3) / x3D_homogeneous(3);
        // std::cout<<"x3D "<<x3D<<std::endl;
        
        // 3.1 check depth
        double z1 = x3D.z();
        if (z1 < 0.0) {
            return std::make_pair(-2, 100);
        }
        // 目的是检查通过三角化求得的三维点 x3D 是否在第二个相机的前方。如果 x3D 的深度 z2 小于 0，意味着该点在相机后方，三角化的结果无效，函数返回一个错误标记
        double z2 = R21.row(2).dot(x3D) + t21.z();
        if (z2 < 0.0) {
            return std::make_pair(-3, 100);
        }

        // uncertainty
        double uncertainty = 0.0;
        // r1 是从相机1的视点出发的射线方向（单位向量）。
        // z1 是三角化后在相机1坐标系中的深度。
        // t 是相机1到相机2的平移向量。
        // 这个公式计算的是三维点 x3D 在相机2坐标系中的坐标。r1 * z1 得到三维点在相机1坐标系下的实际位置，然后减去 t 转换到相机2坐标系。
        Eigen::Vector3d a = r1 * z1 - t;
        // std::cout<<"a:"<<a<<std::endl;
        double t_norm = t.norm();
        double a_norm = a.norm();
        // 角度 alpha 和 beta 用于表示三维重建中两个相机观测的三角形的形状。这些角度在计算深度不确定性时非常有用，因为误差在这些角度较小（射线几乎平行）时更大。
        // Alpha: 表示相机1到相机2的基线方向和相机1观测方向之间的夹角。
        // Beta: 表示相机2观测方向和相机1到相机2的基线方向之间的夹角。
        double alpha = acos(r1.dot(t) / (t_norm * r1.norm()));
        double beta = acos(a.dot(-t) / (a_norm * t_norm));
        // kp2 是在相机2中的关键点（2D坐标）。kp2.norm() 计算关键点的欧几里得范数（即其在图像坐标系中的距离原点的距离）。
        double uv_norm = kp2.norm();
        // 假设图像的像素误差为1个像素，计算由此引起的角度误差。
        // camera2->fx() 是相机2的焦距（单位：像素）。
        // atan((uv_norm + 1) / (camera2->fx())) 计算关键点在 uv_norm + 1 位置（即多1个像素偏移）处的视角（用反正切函数计算）。
        // atan(uv_norm / (camera2->fx())) 计算当前关键点 uv_norm 的视角。
        // 两者的差值 px_error_angle 代表1个像素偏移带来的角度变化。
        double px_error_angle = atan((uv_norm + 1) / (camera2->fx())) - atan(uv_norm / (camera2->fx()));
        // beta_plus 是考虑像素误差后的角度 beta。它增加了 px_error_angle，即表示因像素误差带来的视角变化。
        double beta_plus = beta + px_error_angle;
        // 根据三角形内角和定理，三角形的三个内角之和为 π（180度）。
        // gamma_plus 是新的三角形中第三个角度，等于 π 减去 alpha 和 beta_plus 的和。
        double gamma_plus = M_PI-alpha-beta_plus;
        // 该公式计算在考虑角度误差后的新的三维点深度 z_plus。
        double z_plus = t_norm * sin(beta_plus) / sin(gamma_plus);
        uncertainty = (z_plus - z1);
        //std::cout<<"uncertainty:"<<uncertainty<<std::endl;
        return std::make_pair(z1, uncertainty);
    }

    std::vector<std::pair<double, double>> Triangulation::multiview_triangulate(std::shared_ptr<Frame> frame,
                                                                                std::vector<int> ids,
                                                                                std::vector<std::shared_ptr<Frame>> c_frames,
                                                                                std::vector<std::vector<int>> matches)
    {
        std::vector<std::pair<double, double>> results;
        results.resize(ids.size(), std::make_pair(-1, 100));

        std::vector<int> triangulate_ids;
        Eigen::Vector3d t0 = frame->get_t();
        std::vector<double> distances;

        // 1. distance between frames
        for (int i = 0; i < c_frames.size(); i++) {
            Eigen::Vector3d t1 = c_frames[i]->get_t();
            distances.emplace_back((t0 - t1).norm());
            triangulate_ids.emplace_back(i);
        }
        // sort by distances, high to low
        std::sort(triangulate_ids.begin(), triangulate_ids.end(), [&distances](int i, int j) {
            return distances[i] > distances[j];
        });

        for (int i = 0; i< ids.size(); i ++) {
            std::vector<int> match = matches[i];
            for (const auto& id:triangulate_ids) {
                if (match[id] > 0) {
                    Eigen::Matrix3d R = frame->get_R().transpose() * c_frames[id]->get_R();
                    Eigen::Vector3d t = frame->get_R().transpose() * (c_frames[id]->get_t() - frame->get_t());
                    results[i] = triangulate(frame->get_camera(), c_frames[id]->get_camera(),
                                             frame->features_uv[ids[i]], c_frames[id]->features_uv[match[id]],
                                             R, t);
                }
            }
        }
        return results;
    }

}
