#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Geometry>

// copy Odometry to q/t
inline void OdometryToTransform(const nav_msgs::Odometry& odometry,
                                Eigen::Quaterniond &q_o_c, Eigen::Vector3d &t_o_c){
  q_o_c.x() = odometry.pose.pose.orientation.x;
  q_o_c.y() = odometry.pose.pose.orientation.y;
  q_o_c.z() = odometry.pose.pose.orientation.z;
  q_o_c.w() = odometry.pose.pose.orientation.w;
  t_o_c.x() = odometry.pose.pose.position.x;
  t_o_c.y() = odometry.pose.pose.position.y;
  t_o_c.z() = odometry.pose.pose.position.z;
}
// inline void OdometryToTransform(const nav_msgs::Odometry& odometry, vn::geometry::Pose3D& pose){
//   auto&& p = odometry.pose.pose.position;
//   auto&& q = odometry.pose.pose.orientation;
//   pose.setTranslation(Eigen::Vector3d(p.x, p.y, p.z));
//   pose.setRotation(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
// }
inline void TransformToPoseMsg(const Eigen::Quaterniond &q_o_c, const Eigen::Vector3d &t_o_c,
                                geometry_msgs::Pose& pose) {
  pose.orientation.x= q_o_c.x() ;
  pose.orientation.y= q_o_c.y() ;
  pose.orientation.z= q_o_c.z() ;
  pose.orientation.w= q_o_c.w() ;
  pose.position.x  = t_o_c.x() ;
  pose.position.y  = t_o_c.y() ;
  pose.position.z  = t_o_c.z() ;
}
inline void TransformToOdometry(const Eigen::Quaterniond &q_o_c, const Eigen::Vector3d &t_o_c,
                                nav_msgs::Odometry& odometry){
  TransformToPoseMsg(q_o_c, t_o_c, odometry.pose.pose);
}
// inline void TransformToOdometry(const vn::geometry::Pose3D &pose,
//                                 nav_msgs::Odometry& odometry){
//   TransformToOdometry(pose.rotation(), pose.translation(), odometry);
// }

