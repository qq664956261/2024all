#include "mono_slam/system.h"

#include "DBoW3/DBoW3.h"
#define ARMA_ALLOW_FAKE_CLANG
#include "armadillo"
#include "mono_slam/camera.h"
#include "mono_slam/utils/math_utils.h"

using DBoW3::Vocabulary;

namespace mono_slam {

// Forward declaration.
class Camera;

double Camera::fx_, Camera::fy_, Camera::cx_, Camera::cy_;
Mat33 Camera::K_;
Vec4 Camera::dist_coeffs_;

System::System(const string& config_file) : config_file_(config_file) {}

bool System::init() {
  LOG(INFO) << "System is initializing ...";
  // Read settings from configuration file.
  cv::FileStorage config(config_file_, cv::FileStorage::READ);
  // cv::FileStorage config(
  //     "/home/bayes/Documents/monocular_vo/app/config_kitti.yaml",
  //     cv::FileStorage::READ);
  if (!config.isOpened()) {
    config.release();
    LOG(FATAL) << "Unable to read configuration file.";
  }

  // Prepare dataset.
  const string& dataset_path = config["dataset_path"];
  const string& img_file_name_fmt = config["img_file_name_fmt"];
  const double& img_resize_factor = config["img_resize_factor"];
  const int& img_start_idx = config["img_start_idx"];
  dataset_.reset(new Dataset(dataset_path, img_file_name_fmt, img_resize_factor,
                             img_start_idx));

  // Load vocabulary.
  const string& voc_file = config["voc_file"];
  const steady_clock::time_point t1 = steady_clock::now();
  sptr<Vocabulary> voc = make_shared<Vocabulary>(voc_file);
  // sptr<Vocabulary> voc = make_shared<Vocabulary>(
  //     "/home/bayes/Documents/monocular_vo/data/vocabulary/orbvoc.dbow3");
  const steady_clock::time_point t2 = steady_clock::now();
  const double time_span = duration_cast<duration<double>>(t2 - t1).count();
  LOG(INFO) << "Loaded vocabulary in " << time_span << "seconds.";

  // Load ground truth poses.
  const string& pose_file = config["pose_file"];
  // const string& pose_file{
  //     "/home/bayes/Documents/monocular_vo/data/pose/parking_pose.txt"};
  arma::mat pose_mat;
  if (pose_file.empty())
    LOG(WARNING) << "No ground truth file.";
  else
    pose_mat.load(pose_file, arma::file_type::auto_detect, true);
  // Convert to vector<SE3>.

  pose_ground_truths_.reserve(pose_mat.n_rows);
  for (int i = 0; i < pose_mat.n_rows; ++i)
    pose_ground_truths_.push_back(math_utils::read_pose(pose_mat.row(i)));

  // Load timestamps.
  const string& timestamp_file = config["timestamp_file"];
  arma::mat timestamps_mat;
  if (timestamp_file.empty())
    LOG(WARNING) << "No timestamp file.";
  else
    timestamps_mat.load(timestamp_file, arma::file_type::auto_detect, true);
  timestamps_ = arma::conv_to<vector<double>>::from(timestamps_mat);

  if (pose_ground_truths_.size() != timestamps_.size())
    LOG(WARNING) << "Size of ground truth pose and that of timestamps are not "
                    "consistent.";

  // Set camera parameters.
  const double& fx = config["fx"];
  const double& fy = config["fy"];
  const double& cx = config["cx"];
  const double& cy = config["cy"];
  const double& k1 = config["k1"];
  const double& k2 = config["k2"];
  const double& p1 = config["p1"];
  const double& p2 = config["p2"];
  const Vec4 dist_coeffs{k1, k2, p1, p2};
  Camera::fx_ = fx;
  Camera::fy_ = fy;
  Camera::cx_ = cx;
  Camera::cy_ = cy;
  Camera::K_ = (Mat33() << fx, 0., cx, 0., fy, cy, 0., 0., 1.).finished();
  Camera::dist_coeffs_ = dist_coeffs;

  // Get camera fps.
  const double& fps = config["fps"];

  // Release the file as soon as possible.
  config.release();

  // Set viewer pose.
  const double view_point_dist = 0.3;
  const Vec3f t{0.5 * view_point_dist, -1. * view_point_dist,
                -1 * view_point_dist};
  const Vec3f rotation_vec{-0.5, 0., 0.};  // Look downward.
  const Mat33f R =
      Eigen::AngleAxisf(rotation_vec.norm(), rotation_vec.normalized())
          .toRotationMatrix();
  Eigen::Affine3f viewer_pose;
  viewer_pose.linear() = R;
  viewer_pose.translation() = t;

  // Prepare and link system components.
  tracker_.reset(new Tracking());
  local_mapper_.reset(new LocalMapping());
  map_.reset(new Map(voc));
  viewer_.reset(new Viewer(viewer_pose, fps));

  tracker_->setSystem(shared_from_this());
  tracker_->setLocalMapper(local_mapper_);
  tracker_->setMap(map_);
  tracker_->setViewer(viewer_);
  tracker_->voc_ = voc;

  local_mapper_->setSystem(shared_from_this());
  local_mapper_->setTracker(tracker_);
  local_mapper_->setMap(map_);

  viewer_->setTracker(tracker_);
  viewer_->setMap(map_);

  LOG(INFO) << "System is finished ...";

  return true;
}

void System::run() {
  local_mapper_->startThread();
  viewer_->startThread();
  // Hold on 1 seconds to make the threads ready.
  //! It's just my wishful thinking though.
  std::this_thread::sleep_for(duration<double>(1.0));
  LOG(INFO) << "Tracker is running ...";
  // If timestamp file is not provided, the tracking is performed without any
  // delay.
  // TODO(bayes) Catch empty image exception and exit system.
  if (timestamps_.empty())
    for (;;) tracker_->addImage(dataset_->nextImage());
  else {  // Otherwise, necessary time delay is adopted.
    const int n_images = timestamps_.size();
    // Simply discard the last image for the sake of simplicity.
    for (int i = 0; i < n_images - 1; ++i) {
      const steady_clock::time_point t1 = steady_clock::now();

      // Track one image.
      tracker_->addImage(dataset_->nextImage());

      const steady_clock::time_point t2 = steady_clock::now();
      const double consumed_time =
          duration_cast<duration<double>>(t2 - t1).count();

      // (Only) pause the tracking thread to align the time.
      const double delta_t = timestamps_[i + 1] - timestamps_[i];
      if (consumed_time < delta_t)
        std::this_thread::sleep_for(duration<double>(delta_t - consumed_time));
    }
  }
  LOG(INFO) << "Exit system.";
}

void System::reset() {
  LOG(INFO) << "Resetting system ...";
  tracker_->reset();
  local_mapper_->reset();
  map_->clear();
  viewer_->reset();
  LOG(INFO) << "Reset system.";
}

}  // namespace mono_slam
