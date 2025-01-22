
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

struct uwbFactor
{
	uwbFactor(Eigen::Vector3d point_a, Eigen::Vector3d point_b, double range)
		: point_a_(point_a), point_b_(point_b), range_(range){}

	template <typename T>
	bool operator()(const T *t, T *residual) const
	{
		Eigen::Matrix<T, 3, 1> p{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> pa{T(point_a_[0]), T(point_a_[1]), T(point_a_[2])};
		Eigen::Matrix<T, 3, 1> pb{T(point_b_[0]), T(point_b_[1]), T(point_b_[2])};
		Eigen::Matrix<T, 3, 1> diff_i = pa - p;
        Eigen::Matrix<T, 3, 1> diff_j = pb - p;
		Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals(residual);
		T res = diff_j.norm() - diff_i.norm() - T(range_);
		residuals(0, 0) = res;


		Eigen::Matrix<T, 1, 1> sqrt_info = T(100) * Eigen::Matrix<T, 1, 1>::Identity();//eskf 1 graph 100


		residuals.applyOnTheLeft(sqrt_info);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d point_a, const Eigen::Vector3d point_b, const double range)
	{
		return (new ceres::AutoDiffCostFunction<
				uwbFactor, 1, 3>(//eskf1，3  graph 1,7
			new uwbFactor(point_a, point_b,range)));
	}

	Eigen::Vector3d point_a_;
	Eigen::Vector3d point_b_;
    double range_;
};