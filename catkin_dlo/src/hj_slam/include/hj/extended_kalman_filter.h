// created by: zhangcheng
#ifndef EXTENDED_KALMAN_FILTER_HH
#define EXTENDED_KALMAN_FILTER_HH
#include "Odometry.h"
#include "common.h"
#include <Eigen/Dense>
#include <queue>

struct OdomInfo {
    Odometry odom;
    Eigen::VectorXd x;
};

class ExtendedKalmanFilter {
public:
    //2d
    void Initialize(Pose pose, double time) {
        Eigen::MatrixXd x(3, 1);
        double angle = pose.yaw;
        x << pose.position.x(), pose.position.y(), angle;
        x_ = x;

        odomTimeStamp_ = time;
        Eigen::MatrixXd F(3, 3);
        F << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetF(F);
        Eigen::MatrixXd FJ(3, 3);
        FJ << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetFJ(FJ);

        Eigen::MatrixXd P(3, 3);
        P << 1, 0, 0,
            0, 1, 0,
            0, 0, 100;
        SetP(P);

        Eigen::MatrixXd Q(3, 3);
        Q << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetQ(Q);

        Eigen::MatrixXd H(3, 3);
        H << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetH(H);

        Eigen::MatrixXd R(3, 3);
        R << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetR(R);
        is_initialized_ = true;

        // std::string ekf_path = m_current_facory + "/" + "ekfpredict.csv";
        //  m_predictcsv = std::ofstream(ekf_path.c_str(), std::ios::trunc);
    }
    void SetOdomAngle(double angle_in) {
        angle_ = angle_in;
    }
    void SetF(Eigen::MatrixXd &F_in) {
        F_ = F_in;
    }

    void SetFJ(Eigen::MatrixXd &FJ_in) {
        FJ_ = FJ_in;
    }

    void SetP(Eigen::MatrixXd &P_in) {
        P_ = P_in;
    }

    void SetQ(Eigen::MatrixXd &Q_in) {
        Q_ = Q_in;
    }

    void SetH(Eigen::MatrixXd &H_in) {
        H_ = H_in;
    }

    void SetR(Eigen::MatrixXd &R_in) {
        R_ = R_in;
    }

    Eigen::MatrixXd GetX() {
        return x_;
    }

    bool IsInitialized() {
        return is_initialized_;
    }

    void ReInitialize() {
        is_initialized_ = false;
        x_ = Eigen::Vector3d::Zero();
        F_ = Eigen::Matrix3d::Zero();
        FJ_ = Eigen::Matrix3d::Zero();
        P_ = Eigen::Matrix3d::Zero();
        Q_ = Eigen::Matrix3d::Zero();
        H_ = Eigen::Matrix3d::Zero();
        R_ = Eigen::Matrix3d::Zero();
        angle_ = 0;
        odomTimeStamp_ = 0;
        zTimeStamp_ = 0;
    }

    void SetPath(std::string current_path) {
        m_current_facory = current_path;
    }

    void InputOdomMeasurement(Pose pose) {
        Eigen::Vector3d u_(pose.position.x(), pose.position.y(), pose.yaw);
        Eigen::MatrixXd Fj(3, 3);
        Fj << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
        SetFJ(Fj);
        Prediction(u_);
    }

    void InputOdomMeasurement(Odometry odom) {
        if (odom.header.stamp > 1) {
            auto dt = (odom.header.stamp - odomTimeStamp_) * 1e-6;
            double dz = odom.twist.angular.z() * dt;
            dz = NormalizeAngle(dz);

            // m_predictcsv << odom.pose_odom_agv.translation().x() << "," << odom.pose_odom_agv.translation().y() << "," << odom.pose_odom_agv.getEulerAngle()[0] << ",";

             //if (dt == 0)
              //   return;

            double angle = x_(2) + dz + angle_;
            double dx = odom.twist.linear.x() * cos(angle) * dt;
            double dy = odom.twist.linear.x() * sin(angle) * dt;



            int64_t lastodomtime = (int64_t)(odomTimeStamp_);
            int64_t curodomtime = (int64_t)(odom.header.stamp);

            Eigen::Vector3d u_(dx, dy, dz);
            Eigen::MatrixXd Fj(3, 3);
            Fj << 1, 0, -u_(1),
                0, 1, u_(0),
                0, 0, 1;
            SetFJ(Fj);
            PredictionOld(u_);
            //            ROS_INFO("x y z dx dt %f %f %f %f %f %f",x_(0),x_(1),x_(2),dx,dy,angle);

        }
        odomTimeStamp_ = odom.header.stamp;
    }

    void AddLidarMeasurement(Pose pose) {
        Eigen::VectorXd z(3, 1);
        z << pose.position.x(), pose.position.y(), pose.yaw;
        MeasurementUpdate(z);
    }

    void AddLidarMeasurementOld(Pose pose) {
        Eigen::VectorXd z(3, 1);
        z << pose.position.x(), pose.position.y(), pose.yaw;
        MeasurementUpdateOld(z);
    }


    void PredictionOld(Eigen::VectorXd u_) {
        x_ = F_ * x_ + u_;
        Eigen::MatrixXd Ft = FJ_.transpose();
        P_ = FJ_ * P_ * Ft + Q_; //Э����


    }

    void Prediction(Eigen::VectorXd pose) {
        Eigen::MatrixXd Q(3, 3);
        static Eigen::VectorXd lastPose = pose;

        auto dx = std::abs(pose(0) - lastPose(0));
        auto dy = std::abs(pose(1) - lastPose(1));
        auto dyaw = std::abs(pose(2) - lastPose(2));
        if (dx < 0.001) {
            dx = 0.001;
        }
        if (dy < 0.001) {
            dy = 0.001;
        }
        if (dyaw < 0.001) {
            dyaw = 0.001;
        }
        Q << Q_(0, 0) * dx, 0, 0,
            0, Q_(1, 1) * dy, 0,
            0, 0, Q_(2, 2) * dyaw;
        x_ = pose;
        lastPose = pose;
        Eigen::MatrixXd Ft = FJ_.transpose();
        P_ = FJ_ * P_ * Ft + Q;
    }

    

    void MeasurementUpdateOld(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H_ * x_;
        y(2) = NormalizeAngle(y(2));

        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; 
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();



        Eigen::VectorXd temp = x_;
        x_ = x_ + (K * y);
        x_(2) = NormalizeAngle(x_(2));


        //        std::cout<< K << std::endl;
        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K * H_) * P_;
    }

    void MeasurementUpdate(const Eigen::VectorXd &z) {
        static Eigen::VectorXd lastPose = z;

        auto dx = std::abs(z(0) - lastPose(0));
        auto dy = std::abs(z(1) - lastPose(1));
        auto dyaw = std::abs(z(2) - lastPose(2));
        if (dx < 0.001) {
            dx = 0.001;
        }
        if (dy < 0.001) {
            dy = 0.001;
        }
        if (dyaw < 0.001) {
            dyaw = 0.001;
        }
        lastPose = z;
        Eigen::MatrixXd R(3, 3);
        R << R_(0, 0) * dx, 0, 0,
            0, R_(1, 1) * dy, 0,
            0, 0, R_(2, 2) * dyaw;
        Eigen::VectorXd y = z - H_ * x_;
        y(2) = NormalizeAngle(y(2));
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        x_(2) = NormalizeAngle(x_(2));
        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K * H_) * P_;
    }

    void Prediction_3D(Eigen::VectorXd pose) {
        Eigen::MatrixXd Q(6, 6);
        static Eigen::VectorXd lastPose = pose;

        auto dx = std::abs(pose(0) - lastPose(0));
        auto dy = std::abs(pose(1) - lastPose(1));
        auto dz = std::abs(pose(2) - lastPose(2));
        auto droll = std::abs(pose(0) - lastPose(0));
        auto dpitch = std::abs(pose(1) - lastPose(1));
        auto dyaw = std::abs(pose(2) - lastPose(2));
        if (dx < 0.001) {
            dx = 0.001;
        }
        if (dy < 0.001) {
            dy = 0.001;
        }
        if (dz < 0.001) {
            dz = 0.001;
        }
        if (droll < 0.001) {
            droll = 0.001;
        }
        if (dpitch < 0.001) {
            dpitch = 0.001;
        }
        if (dyaw < 0.001) {
            dyaw = 0.001;
        }    

         Q << Q_(0, 0) * dx, 0, 0, 0, 0, 0,
               0, Q_(1, 1)* dy, 0, 0, 0, 0,
               0, 0, Q_(2, 2)* dz, 0, 0, 0,
               0, 0, 0, Q_(3, 3)* droll, 0, 0,
               0, 0, 0, 0, Q_(4, 4)* dpitch, 0,
               0, 0, 0, 0, 0, Q_(5, 5)* dyaw;

        x_ = pose;
        lastPose = pose;
        Eigen::MatrixXd Ft = FJ_.transpose();
        P_ = FJ_ * P_ * Ft + Q;
    }

private:
    bool is_initialized_{false};
    Eigen::VectorXd x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd FJ_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    double angle_{ 0.0 };
    double odomTimeStamp_;
    double zTimeStamp_;

    std::string m_current_facory;
    std::ofstream m_predictcsv;
    
    public:
    //3d
    void Initialize3d(Pose pose, double time) {
        Eigen::MatrixXd x(6, 1);
        double yawangle = pose.yaw;
        double rollangle = pose.roll;
        double pitchangle = pose.pitch;

        x << pose.position.x(), pose.position.y(), pose.position.z(), rollangle, pitchangle, yawangle;
        x_3d = x;

        odomTimeStamp_3d = time;
        Eigen::MatrixXd F(6, 6);
        F << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 
            0, 0, 0, 0, 1, 0, 
            0, 0, 0, 0, 0, 1;
        SetF3d(F);
        Eigen::MatrixXd FJ(6, 6);
        FJ << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        SetFJ3d(FJ);

        Eigen::MatrixXd P(6, 6);
        P << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100;
        SetP3d(P);

        Eigen::MatrixXd Q(6, 6);
        Q << 0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 0.001, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0,
            0, 0, 0, 0, 0.001, 0,
            0, 0, 0, 0, 0, 0.001;
        SetQ3d(Q);

        Eigen::MatrixXd H(6, 6);
        H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        SetH3d(H);

        Eigen::MatrixXd R(6, 6);
        R << 0.05, 0, 0, 0, 0, 0,
            0, 0.05, 0, 0, 0, 0,
            0, 0, 0.05, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0.01;
        SetR3d(R);
        is_initialized_ = true;

        // std::string ekf_path = m_current_facory + "/" + "ekfpredict.csv";
        // m_predictcsv = std::ofstream(ekf_path.c_str(), std::ios::trunc);
    }
    void SetOdomAngle3d(double angle_in) {
        angle_ = angle_in;
    }
    void SetF3d(Eigen::MatrixXd& F_in) {
        F_3d = F_in;
    }

    void SetFJ3d(Eigen::MatrixXd& FJ_in) {
        FJ_3d = FJ_in;
    }

    void SetP3d(Eigen::MatrixXd& P_in) {
        P_3d = P_in;
    }

    void SetQ3d(Eigen::MatrixXd& Q_in) {
        Q_3d = Q_in;
    }

    void SetH3d(Eigen::MatrixXd& H_in) {
        H_3d = H_in;
    }

    void SetR3d(Eigen::MatrixXd& R_in) {
        R_3d = R_in;
    }

    Eigen::MatrixXd GetX3d() {
        return x_3d;
    }

    bool IsInitialized3d() {
        return is_initialized_;
    }

    void ReInitialize3d() {
        is_initialized_ = false;
    }

    void SetPath3d(std::string current_path) {
        m_current_facory = current_path;
    }

    void InputOdomMeasurement3d(Pose pose) {
        Eigen::VectorXd u_(6, 1);
        u_ << pose.position.x(), pose.position.y(), pose.position.z(),
            pose.roll, pose.pitch, pose.yaw;

        Eigen::MatrixXd Fj(6, 6);
        Fj <<   1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;
        SetFJ3d(Fj);
        Prediction3d(u_);
    }


    void AddLidarMeasurement3d(Pose pose) {
        Eigen::VectorXd z(6, 1);
        z << pose.position.x(), pose.position.y(), pose.position.z(), pose.roll, pose.pitch, pose.yaw;
        MeasurementUpdate3d(z);
    }

    void Prediction3d(Eigen::VectorXd pose) 
    {
        Eigen::MatrixXd Q(6, 6);
        static Eigen::VectorXd lastPose = pose;

        auto dx = std::abs(pose(0) - lastPose(0));
        auto dy = std::abs(pose(1) - lastPose(1));
        auto dz = std::abs(pose(2) - lastPose(2));
        auto droll = std::abs(pose(0) - lastPose(0));
        auto dpitch = std::abs(pose(1) - lastPose(1));
        auto dyaw = std::abs(pose(2) - lastPose(2));
        if (dx < 0.001) {
            dx = 0.001;
        }
        if (dy < 0.001) {
            dy = 0.001;
        }
        if (dz < 0.001) {
            dz = 0.001;
        }
        if (droll < 0.001) {
            droll = 0.001;
        }
        if (dpitch < 0.001) {
            dpitch = 0.001;
        }
        if (dyaw < 0.001) {
            dyaw = 0.001;
        }

        Q << Q_3d(0, 0) * dx, 0, 0, 0, 0, 0,
            0, Q_3d(1, 1)* dy, 0, 0, 0, 0,
            0, 0, Q_3d(2, 2)* dz, 0, 0, 0,
            0, 0, 0, Q_3d(3, 3)* droll, 0, 0,
            0, 0, 0, 0, Q_3d(4, 4)* dpitch, 0,
            0, 0, 0, 0, 0, Q_3d(5, 5)* dyaw;
        
        x_3d = pose;
        lastPose = pose;
        Eigen::MatrixXd Ft = FJ_3d.transpose();

    

        P_3d = FJ_3d * P_3d * Ft + Q;



    }


    void MeasurementUpdate3d(const Eigen::VectorXd& z) {
        static Eigen::VectorXd lastPose = z;

        auto dx = std::abs(z(0) - lastPose(0));
        auto dy = std::abs(z(1) - lastPose(1));
        auto dz = std::abs(z(2) - lastPose(2));
        auto droll = std::abs(z(3) - lastPose(3));
        auto dpitch = std::abs(z(4) - lastPose(4));
        auto dyaw = std::abs(z(5) - lastPose(5));

        if (dx < 0.001) {
            dx = 0.001;
        }

        if (dy < 0.001) {
            dy = 0.001;
        }

        if (dz < 0.001) {
            dz = 0.001;
        }

        if (dyaw < 0.001) {
            dyaw = 0.001;
        }

        if (droll < 0.001) {
            droll = 0.001;
        }

        if (dpitch < 0.001) {
            dpitch = 0.001;
        }

        lastPose = z;
        Eigen::MatrixXd R(6, 6);
        R.setZero();
        R << R_3d(0, 0) * dx, 0, 0, 0, 0, 0,
            0, R_3d(1, 1)* dy, 0, 0, 0, 0,
            0, 0, R_3d(2, 2)* dz, 0, 0, 0,
            0, 0, 0, R_3d(3, 3)* droll, 0, 0,
            0, 0, 0, 0, R_3d(4, 4)* dpitch, 0,
            0, 0, 0, 0, 0, R_3d(5, 5)* dyaw;

        Eigen::VectorXd y = z - H_3d * x_3d;

        y(3) = NormalizeAngle(y(3));
        y(4) = NormalizeAngle(y(4));
        y(5) = NormalizeAngle(y(5));

        Eigen::MatrixXd S;
        S.setZero();
        S = H_3d * P_3d * H_3d.transpose() + R;
        Eigen::MatrixXd K;
        K.setZero();
        K = P_3d * H_3d.transpose() * S.inverse();

        Eigen::MatrixXd S_inv = S.inverse();

        
        Eigen::VectorXd K_y = K * y;
        if (std::isnan(K_y(0)) || std::isnan(K_y(1)) || std::isnan(K_y(2))
            || std::isnan(K_y(3)) || std::isnan(K_y(4)) || std::isnan(K_y(5)))
        {
            
            x_3d = x_3d;
        }
        else
        {
            x_3d = x_3d + K_y;
            
            x_3d(3) = NormalizeAngle(x_3d(3));
            x_3d(4) = NormalizeAngle(x_3d(4));
            x_3d(5) = NormalizeAngle(x_3d(5));
            int size = x_3d.size();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
            P_3d = (I - K * H_3d) * P_3d;

        }



        
    }

    private:

        Eigen::VectorXd x_3d;
        Eigen::MatrixXd F_3d;
        Eigen::MatrixXd FJ_3d;
        Eigen::MatrixXd P_3d;
        Eigen::MatrixXd Q_3d;
        Eigen::MatrixXd H_3d;
        Eigen::MatrixXd R_3d;

        double angle_3d{0.0};
        double odomTimeStamp_3d;
        double zTimeStamp_3d;

        std::string m_current_facory_3d;
        std::ofstream m_predictcsv_3d;
};
#endif // LOCALIZE_EXTENDED_KALMAN_FILTER_HH