
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

struct consecutivePose
{
	consecutivePose(Eigen::Vector4f q_constraint_, Eigen::Vector3d t_constraint_)
		: q_constraint(q_constraint_), t_constraint(t_constraint_) {}

	template <typename T>
	bool operator()(const T *T1, const T *T2, T *residual) const
	{
		Eigen::Quaternion<T> quaternion1{T1[6], T1[3], T1[4], T1[5]};
		Eigen::Matrix<T, 3, 1> translation1{T1[0], T1[1], T1[2]};
		Eigen::Quaternion<T> quaternion2{T2[6], T2[3], T2[4], T2[5]};
		Eigen::Matrix<T, 3, 1> translation2{T2[0], T2[1], T2[2]};
		Eigen::Quaternion<T> quaternion3{T(q_constraint[3]), T(q_constraint[0]), T(q_constraint[1]), T(q_constraint[2])};
		Eigen::Matrix<T, 3, 1> translation3{T(t_constraint[0]), T(t_constraint[1]), T(t_constraint[2])};
		Eigen::Quaternion<T> relative_q;
		Eigen::Matrix<T, 3, 1> relative_t;
		relative_q = quaternion2.inverse() * quaternion1;
		relative_t = quaternion2.inverse() * (translation1 - translation2);
		Eigen::Matrix<T, 3, 1> residual_q;
		residual_q = (relative_q * quaternion3.inverse()).vec();

		Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);
		residuals.template block<3, 1>(0, 0) = relative_t - translation3;
		residuals.template block<3, 1>(3, 0) = T(30) * residual_q;
		//Eigen::Matrix<T, 6, 6> sqrt_info = T(100) * Eigen::Matrix<T, 6, 6>::Identity();
		Eigen::Matrix<T, 6, 6> sqrt_info = T(500) * Eigen::Matrix<T, 6, 6>::Identity();

		residuals.applyOnTheLeft(sqrt_info);


		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector4f q_constraint_, const Eigen::Vector3d t_constraint_)
	{
		return (new ceres::AutoDiffCostFunction<
				consecutivePose, 6,7,7>(
			new consecutivePose(q_constraint_, t_constraint_)));
	}
	Eigen::Vector4f q_constraint;
	Eigen::Vector3d t_constraint;
};





