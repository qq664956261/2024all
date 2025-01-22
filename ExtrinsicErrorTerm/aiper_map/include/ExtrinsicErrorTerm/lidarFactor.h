
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>

inline Eigen::Matrix3d skewd(const Eigen::Vector3d &x)
{
	Eigen::Matrix3d skew1 = Eigen::Matrix3d::Zero();
	skew1(0, 1) = -x[2];
	skew1(0, 2) = x[1];
	skew1(1, 0) = x[2];
	skew1(1, 2) = -x[0];
	skew1(2, 0) = -x[1];
	skew1(2, 1) = x[0];

	return skew1;
}
struct sonarEdgeFactor
{
	sonarEdgeFactor(Eigen::Vector3d point_a, Eigen::Vector3d point_b, Eigen::Quaterniond wq, Eigen::Vector3d wt)
		: point_a_(point_a), point_b_(point_b), wq_(wq), wt_(wt) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_12{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_12{t[0], t[1], t[2]};
		Eigen::Quaternion<T> q_corrected{T(wq_.w()), T(wq_.x()), T(wq_.y()), T(wq_.z())};
		Eigen::Matrix<T, 3, 1> t_corrected{T(wt_[0]), T(wt_[1]), T(wt_[2])};
		Eigen::Matrix<T, 3, 1> pa{T(point_a_[0]), T(point_a_[1]), T(point_a_[2])};
		Eigen::Matrix<T, 3, 1> pb{T(point_b_[0]), T(point_b_[1]), T(point_b_[2])};
		Eigen::Matrix<T, 3, 1> pc;
		pc = q_corrected * q_12 * pa + q_corrected * t_12 + t_corrected;
		Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(residual);
		residuals.template block<3, 1>(0, 0) = pc - pb;

		// residual[0] = (pc.x() - pb.x());
		// residual[1] = pc.y() - pb.y();
		// residual[2] = pc.z() - pb.z();
		Eigen::Matrix<T, 3, 3> sqrt_info = T(50) * Eigen::Matrix<T, 3, 3>::Identity();


		residuals.applyOnTheLeft(sqrt_info);

		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d point_a, const Eigen::Vector3d point_b, const Eigen::Quaterniond wq, const Eigen::Vector3d wt)
	{
		return (new ceres::AutoDiffCostFunction<
				sonarEdgeFactor, 3, 4, 3>(
			new sonarEdgeFactor(point_a, point_b, wq, wt)));
	}

	Eigen::Vector3d point_a_;
	Eigen::Vector3d point_b_;
	Eigen::Quaterniond wq_;
	Eigen::Vector3d wt_;
};


struct consecutivePose
{
	consecutivePose(Eigen::Vector4f q_constraint_, Eigen::Vector3d t_constraint_)
		: q_constraint(q_constraint_), t_constraint(t_constraint_) {}

	template <typename T>
	bool operator()(const T *q1, const T *t1, const T *q2, const T *t2, T *residual) const
	{
		Eigen::Quaternion<T> quaternion1{q1[3], q1[0], q1[1], q1[2]};
		Eigen::Matrix<T, 3, 1> translation1{t1[0], t1[1], t1[2]};
		Eigen::Quaternion<T> quaternion2{q2[3], q2[0], q2[1], q2[2]};
		Eigen::Matrix<T, 3, 1> translation2{t2[0], t2[1], t2[2]};
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
		Eigen::Matrix<T, 6, 6> sqrt_info = T(100) * Eigen::Matrix<T, 6, 6>::Identity();

		residuals.applyOnTheLeft(sqrt_info);


		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector4f q_constraint_, const Eigen::Vector3d t_constraint_)
	{
		return (new ceres::AutoDiffCostFunction<
				consecutivePose, 6, 4, 3, 4, 3>(
			new consecutivePose(q_constraint_, t_constraint_)));
	}
	Eigen::Vector4f q_constraint;
	Eigen::Vector3d t_constraint;
};

struct loopPose
{
	loopPose(Eigen::Vector4f q_constraint_, Eigen::Vector3d t_constraint_)
		: q_constraint(q_constraint_), t_constraint(t_constraint_) {}

	template <typename T>
	bool operator()(const T *q1, const T *t1, const T *q2, const T *t2, T *residual) const
	{
		Eigen::Quaternion<T> quaternion1{q1[3], q1[0], q1[1], q1[2]};
		Eigen::Matrix<T, 3, 1> translation1{t1[0], t1[1], t1[2]};
		Eigen::Quaternion<T> quaternion2{q2[3], q2[0], q2[1], q2[2]};
		Eigen::Matrix<T, 3, 1> translation2{t2[0], t2[1], t2[2]};
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
		Eigen::Matrix<T, 6, 6> sqrt_info = T(1000) * Eigen::Matrix<T, 6, 6>::Identity();

		residuals.applyOnTheLeft(sqrt_info);


		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector4f q_constraint_, const Eigen::Vector3d t_constraint_)
	{
		return (new ceres::AutoDiffCostFunction<
				loopPose, 6, 4, 3, 4, 3>(
			new loopPose(q_constraint_, t_constraint_)));
	}
	Eigen::Vector4f q_constraint;
	Eigen::Vector3d t_constraint;
};



