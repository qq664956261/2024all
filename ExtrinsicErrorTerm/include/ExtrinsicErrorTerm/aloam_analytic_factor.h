#ifndef LIDAR_LOCALIZATION_MODELS_ALOAM_ANALYTIC_FACTOR_HPP_
#define LIDAR_LOCALIZATION_MODELS_ALOAM_ANALYTIC_FACTOR_HPP_
#include <eigen3/Eigen/Dense>
#include <sophus/so3.h>
#include <sophus/se3.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

static Eigen::Matrix<double, 3, 3> skew(Eigen::Matrix<double, 3, 1> &mat_in)
{ //  反对称矩阵定义
        Eigen::Matrix<double, 3, 3> skew_mat;
        skew_mat.setZero();
        skew_mat(0, 1) = -mat_in(2);
        skew_mat(0, 2) = mat_in(1);
        skew_mat(1, 2) = -mat_in(0);
        skew_mat(1, 0) = mat_in(2);
        skew_mat(2, 0) = -mat_in(1);
        skew_mat(2, 1) = mat_in(0);
        return skew_mat;
}

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{ // 优化参数维度：1     输入维度 ： q : 4   t : 3
public:
        double s;
        Eigen::Vector3d curr_point, last_point_a, last_point_b;
        Eigen::Quaterniond wq;
        Eigen::Vector3d wt;
        EdgeAnalyticCostFunction(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                 const Eigen::Vector3d last_point_b_, const double s_, Eigen::Quaterniond wq_, Eigen::Vector3d wt_)
            : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_), wq(wq_), wt(wt_) {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const //   定义残差模型
        {
                Eigen::Quaternion<double> q_12{parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]};
                Eigen::Matrix<double, 3, 1> t_12{parameters[1][0], parameters[1][1], parameters[1][2]};

                Eigen::Quaterniond q_corrected{wq.w(), wq.x(), wq.y(), wq.z()};
                Eigen::Matrix<double, 3, 1> t_corrected{wt[0], wt[1], wt[2]};
                Eigen::Matrix3d R_w_front(q_corrected);
                Eigen::Vector3d lp; //   line point
                Eigen::Vector3d lp_r;
                lp_r = q_12 * curr_point;
                lp = q_corrected * q_12 * curr_point + q_corrected * t_12 + t_corrected; //   new point
                Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
                Eigen::Vector3d de = last_point_a - last_point_b;
                Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);
                //residuals[0] = nu.norm() / de.norm(); //  线残差
                Eigen::Matrix<double, 1, 1> res;
                res << nu.norm() / de.norm();
                		
		residual.template block<1, 1>(0, 0) = res;

	
		Eigen::Matrix<double, 1, 1> sqrt_info = 50 * Eigen::Matrix<double, 1, 1>::Identity();

		residual.applyOnTheLeft(sqrt_info);

                //  归一单位化
                nu.normalize();

                if (jacobians != NULL)
                {
                        if (jacobians[0] != NULL)
                        {
                                Eigen::Vector3d re = last_point_b - last_point_a;
                                Eigen::Matrix3d skew_re = skew(re);

                                //  J_so3_Rotation
                                Eigen::Matrix3d skew_lp_r = R_w_front * -skew(lp_r);
                                Eigen::Matrix3d dp_by_dr;
                                dp_by_dr.block<3, 3>(0, 0) = skew_lp_r;
                                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                                J_so3_r.setZero();
                                J_so3_r.block<1, 3>(0, 0) = nu.transpose() * skew_re * dp_by_dr / (de.norm() * nu.norm());

                                //  J_so3_Translation
                                Eigen::Matrix3d dp_by_dt;
                                (dp_by_dt.block<3, 3>(0, 0)).setIdentity();
                                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
                                J_so3_t.setZero();
                                J_so3_t.block<1, 3>(0, 0) = nu.transpose() * skew_re * R_w_front / (de.norm() * nu.norm());
                        }
                }
                return true;
        }
};

class PlaneAnalyticCostFunction : public ceres::SizedCostFunction<1, 4, 3>
{
public:
        Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
        Eigen::Vector3d ljm_norm;
        Eigen::Quaterniond wq;
        Eigen::Vector3d wt;
        double s;

        PlaneAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                                  Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_, Eigen::Quaterniond wq_, Eigen::Vector3d wt_)
            : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_), wq(wq_), wt(wt_) {}

        virtual bool Evaluate(double const *const *parameters,
                              double *residuals,
                              double **jacobians) const
        { //   定义残差模型
                // 叉乘运算， j,l,m 三个但构成的平行四边面积(摸)和该面的单位法向量(方向)
                Eigen::Vector3d ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
                ljm_norm.normalize(); //  单位法向量

                Eigen::Quaternion<double> q_12{parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]};
                Eigen::Matrix<double, 3, 1> t_12{parameters[1][0], parameters[1][1], parameters[1][2]};

                Eigen::Quaterniond q_corrected{wq.w(), wq.x(), wq.y(), wq.z()};
                Eigen::Matrix<double, 3, 1> t_corrected{wt[0], wt[1], wt[2]};
                Eigen::Matrix3d R_w_front(q_corrected);
                Eigen::Vector3d lp;
                Eigen::Vector3d lp_r = q_12 * curr_point; //  for compute jacobian o rotation  L: dp_dr
                lp = q_corrected * q_12 * curr_point + q_corrected * t_12 + t_corrected;

                // 残差函数
                double phi1 = (lp - last_point_j).dot(ljm_norm);

                residuals[0] = std::fabs(phi1);

                if (jacobians != NULL)
                {
                        if (jacobians[0] != NULL)
                        {
                                phi1 = phi1 / residuals[0];
                                //  Rotation
                                Eigen::Matrix3d skew_lp_r = R_w_front * -skew(lp_r);
                                Eigen::Matrix3d dp_dr;
                                dp_dr.block<3, 3>(0, 0) = skew_lp_r;
                                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                                J_so3_r.setZero();
                                // J_so3_r.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose() * (dp_dr);
                                J_so3_r.block<1, 3>(0, 0) = ljm_norm.transpose() * (dp_dr);

                                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_so3_t(jacobians[1]);
                                // J_so3_t.block<1, 3>(0, 0) = phi1 * ljm_norm.transpose() * R_w_front;
                                J_so3_t.block<1, 3>(0, 0) = ljm_norm.transpose() * R_w_front;
                        }
                }
                return true;
        }
};

//  自定义旋转残差块
class PoseSO3Parameterization : public ceres::LocalParameterization
{ //  自定义so3  旋转块
public:
        PoseSO3Parameterization() {}

        virtual ~PoseSO3Parameterization() {}

        virtual bool Plus(const double *x,
                          const double *delta,
                          double *x_plus_delta) const // 参数正切空间上的更新函数
        {
                Eigen::Map<const Eigen::Quaterniond> quater(x);     //   待更新的四元数
                Eigen::Map<const Eigen::Vector3d> delta_so3(delta); //    delta 值,使用流形 so3 更新

                Eigen::Quaterniond delta_quater = Sophus::SO3::exp(delta_so3).unit_quaternion(); //   so3 转换位 delta_p  四元数

                Eigen::Map<Eigen::Quaterniond> quter_plus(x_plus_delta); //   更新后的四元数

                // 旋转更新公式
                quter_plus = (delta_quater * quater).normalized();

                return true;
        }

        virtual bool ComputeJacobian(const double *x, double *jacobian) const //  四元数对so3的偏导数
        {
                Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
                (j.topRows(3)).setIdentity();
                (j.bottomRows(1)).setZero();

                return true;
        }

        // virtual bool MultiplyByJacobian(const double* x,
        //                                 const int num_rows,
        //                                 const double* global_matrix,
        //                                 double* local_matrix) const;//一般不用

        virtual int GlobalSize() const { return 4; } // 参数的实际维数
        virtual int LocalSize() const { return 3; }  // 正切空间上的参数维数
};

struct EdgeFactor
{
        EdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                   Eigen::Vector3d last_point_b_, Eigen::Quaterniond wq_, Eigen::Vector3d wt_)
            : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), wq(wq_), wt(wt_) {}

        template <typename T>
        bool operator()(const T *q, const T *t, T *residual) const
        {

                Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
                Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
                Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

                // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
                Eigen::Quaternion<T> q_12{q[3], q[0], q[1], q[2]};
                Eigen::Matrix<T, 3, 1> t_12{T(t[0]), T(t[1]), T(t[2])};
                Eigen::Quaternion<T> q_corrected{T(wq.w()), T(wq.x()), T(wq.y()), T(wq.z())};
                Eigen::Matrix<T, 3, 1> t_corrected{T(wt[0]), T(wt[1]), T(wt[2])};

                Eigen::Matrix<T, 3, 1> lp;
                lp = q_corrected * q_12 * cp + q_corrected * t_12 + t_corrected;

                Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
                Eigen::Matrix<T, 3, 1> de = lpa - lpb;

                residual[0] = nu.x() / de.norm();
                residual[1] = nu.y() / de.norm();
                residual[2] = nu.z() / de.norm();

                return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                           const Eigen::Vector3d last_point_b_, const Eigen::Quaterniond wq_, const Eigen::Vector3d wt_)
        {
                return (new ceres::AutoDiffCostFunction<
                        EdgeFactor, 3, 4, 3>(
                    new EdgeFactor(curr_point_, last_point_a_, last_point_b_, wq_, wt_)));
        }

        Eigen::Vector3d curr_point, last_point_a, last_point_b;
        Eigen::Quaterniond wq;
        Eigen::Vector3d wt;
};

#endif
