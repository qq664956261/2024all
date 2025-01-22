
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

struct uwbFactorGraph
{
	uwbFactorGraph(Eigen::Vector3d point_a, Eigen::Vector3d point_b, Eigen::Matrix4d T_base_uwb, double range)
		: point_a_(point_a), point_b_(point_b), T_base_uwb_(T_base_uwb), range_(range){}

	template <typename T>
	bool operator()(const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> p{t[0], t[1], t[2]};
		Eigen::Quaternion<T> q{t[6], t[3], t[4], t[5]};
		Eigen::Matrix<T, 3, 1> pa{T(point_a_[0]), T(point_a_[1]), T(point_a_[2])};
		Eigen::Matrix<T, 3, 1> pb{T(point_b_[0]), T(point_b_[1]), T(point_b_[2])};
		Eigen::Matrix<T, 3, 3> R_base_uwb = T_base_uwb_.block<3, 3>(0, 0).cast<T>();
		Eigen::Matrix<T, 3, 1> t_base_uwb = T_base_uwb_.block<3, 1>(0, 3).cast<T>();
		p = q * t_base_uwb + p;
		Eigen::Matrix<T, 3, 1> diff_i = pa - p;
        Eigen::Matrix<T, 3, 1> diff_j = pb - p;
		Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals(residual);
		T res = diff_j.norm() - diff_i.norm() - T(range_);
		residuals(0, 0) = res;


		Eigen::Matrix<T, 1, 1> sqrt_info = T(10) * Eigen::Matrix<T, 1, 1>::Identity();//eskf 1 simple graph 10


		residuals.applyOnTheLeft(sqrt_info);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d point_a, const Eigen::Vector3d point_b, const Eigen::Matrix4d T_base_uwb, const double range)
	{
		return (new ceres::AutoDiffCostFunction<
				uwbFactorGraph, 1, 7>(//eskf1，3  graph 1,7
			new uwbFactorGraph(point_a, point_b, T_base_uwb, range)));
	}

	Eigen::Vector3d point_a_;
	Eigen::Vector3d point_b_;
	Eigen::Matrix4d T_base_uwb_;
    double range_;
};