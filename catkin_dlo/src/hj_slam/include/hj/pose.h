
// created by: zhangcheng
#ifndef POSE_HH
#define POSE_HH


#include "header.h"

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>


class Pose {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pose() {
        quaternion.setIdentity();
        position.setIdentity();
        matrix.setIdentity();
    };

    Pose(double x_, double y_, double z_, double roll_, double pitch_, double yaw_) :
            roll(roll_), pitch(pitch_),
            yaw(yaw_) {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()));
        quaternion = yawAngle * pitchAngle * rollAngle;
        position = Eigen::Vector3d(x_, y_, z_);
        Eigen::Affine3d T = Eigen::Translation3d(position.x(), position.y(), position.z())
                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        matrix = T.matrix();
    };

    Pose(Eigen::Vector3d &position_, Eigen::Quaterniond &quaternion_) :
            position(position_), quaternion(quaternion_) {
        auto euler = QuaternionToEulerAngles(quaternion);
        roll = euler[0];
        pitch = euler[1];
        yaw = euler[2];
        Eigen::Affine3d T = Eigen::Translation3d(position.x(), position.y(), position.z())
                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        matrix = T.matrix();
    };

    void SetQuaternionRollPitchYaw(double roll_, double pitch_, double yaw_) {

        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()));
        quaternion = yawAngle * pitchAngle * rollAngle;
        auto euler = QuaternionToEulerAngles(quaternion);
        roll = euler[0];
        pitch = euler[1];
        yaw = euler[2];
        Eigen::Affine3d T = Eigen::Translation3d(position.x(), position.y(), position.z())
                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        matrix = T.matrix();
    }

    void SetPosition(Eigen::Vector3d &position_) {
        position = position_;
        auto euler = QuaternionToEulerAngles(quaternion);
        roll = euler[0];
        pitch = euler[1];
        yaw = euler[2];
        Eigen::Affine3d T = Eigen::Translation3d(position.x(), position.y(), position.z())
                            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        std::cout << quaternion.matrix() << std::endl;
        std::cout << position.matrix() << std::endl;
        std::cout << T.matrix() << std::endl;
        matrix = T.matrix();
    }
    void SetMatrix(Eigen::Matrix4d matrix_) {

        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.matrix() = std::move(matrix_);
        matrix = T.matrix();
        quaternion = Eigen::Quaterniond(T.rotation().matrix());
        position.matrix() = T.translation().matrix();
        auto euler = QuaternionToEulerAngles(quaternion);
        roll = euler[0];
        pitch = euler[1];
        yaw = euler[2];
    }
    Eigen::Vector3d QuaternionToEulerAngles(Eigen::Quaterniond quaternion) {
        auto qw = quaternion.w();
        auto qx = quaternion.x();
        auto qy = quaternion.y();
        auto qz = quaternion.z();

        double pitch, roll, yaw; //俯仰角pitch,滚动角roll,航向角yaw
        pitch = atan2f(2.f * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        roll = asinf(2.f * (qw * qy - qz * qx));
        yaw = atan2f(2.f * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

        return Eigen::Vector3d(roll, pitch, yaw);

    }
    Pose operator*(Pose pose){
        Pose out;
        out.SetMatrix(matrix * pose.matrix);
        out.header = header;
        return out;
    }
    Pose inverse(){
        Pose out;
        out.SetMatrix(matrix.inverse());
        out.header = header;
        return out;
    }
    void printPose(std::string name="") const {
        printf("%s : x,y,z : [ %f %f %f], r,p,y : [%f %f %f]\n",name.data(),position.x(),position.y(),position.z(),roll,pitch,yaw);
    }
    Header header;

    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;

    Eigen::Matrix4d matrix;
    double roll{};
    double pitch{};
    double yaw{};
};

#endif // POSE_HH
