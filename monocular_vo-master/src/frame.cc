#include "mono_slam/frame.h"

#include "mono_slam/feature.h"
#include "mono_slam/utils/math_utils.h"
#include "opencv2/calib3d.hpp"  // cv::undistortPoints.
template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}
namespace mono_slam {

int Frame::frame_cnt_ = 0;
double Frame::x_min_, Frame::x_max_, Frame::y_min_, Frame::y_max_;

Frame::Frame(const cv::Mat& img)
    : id_(frame_cnt_++), is_keyframe_(false), is_datum_(false) {
  img.copyTo(img_);
  cam_.reset(new Camera());
  // TODO(bayes) Optimize when no distortion.
  // Compute image bounds (computed once in the first frame).
  if (id_ == 0) {
    // Matrix containing the four corners of the image:
    // Left upper, right upper, left bottom, right bottom.
    cv::Mat corners;
    frame_utils::computeImageBounds(img, cam_->K(), cam_->distCoeffs(),
                                    corners);
    x_min_ = std::min(corners.at<float>(0, 0), corners.at<float>(2, 0));
    x_max_ = std::max(corners.at<float>(1, 0), corners.at<float>(3, 0));
    y_min_ = std::min(corners.at<float>(0, 1), corners.at<float>(1, 1));
    y_max_ = std::max(corners.at<float>(2, 1), corners.at<float>(3, 1));
  }
}

void Frame::setPose(const SE3& T_c_w) {
  lock_g lock(mut_);
  cam_->setPose(T_c_w);
}

void Frame::setKeyframe() {
  lock_g lock(mut_);
  is_keyframe_ = true;
}

vector<int> Frame::searchFeatures(const Vec2& pt, const int radius,
                                  int level_low, int level_high) const {
  // level_low = std::clamp(level_low, 0, Config::scale_n_levels());
  // level_high = std::clamp(level_high, 0, Config::scale_n_levels());
  level_low = clamp(level_low, 0, Config::scale_n_levels());
  level_high = clamp(level_high, 0, Config::scale_n_levels());
  const int num_obs = this->nObs();
  vector<int> feat_indices;
  feat_indices.reserve(num_obs);
  for (int i = 0; i < num_obs; ++i) {
    const sptr<Feature>& feat = feats_[i];
    const Vec2 dist = (feat->pt_ - pt).cwiseAbs();
    if (dist.x() <= radius && dist.y() <= radius)
      if (feat->level_ >= level_low && feat->level_ <= level_high)
        feat_indices.push_back(i);
  }
  return feat_indices;
}

//##############################################################################
// Covisibility graph related.

void Frame::addConnection(Frame::Ptr keyframe, const int weight) {
  LOG(INFO) << cv::format("add_connection(add frame: %d, to frame: %d)",
                          keyframe->id_, id_);
  bool need_update = false;
  {
    lock_g lock(co_mut_);
    if (!co_kf_weights_.count(keyframe) || co_kf_weights_[keyframe] != weight)
      need_update = true;
  }
  if (need_update) updateCoKfsAndWeights();
}

void Frame::deleteConnection(const Frame::Ptr& keyframe) {
  LOG(INFO) << cv::format("delete_connection(delete frame: %d, from frame: %d)",
                          keyframe->id_, id_);
  bool need_update = false;
  {
    lock_g lock(co_mut_);
    if (co_kf_weights_.count(keyframe)) {
      co_kf_weights_.erase(keyframe);
      need_update = true;
    }
  }
  if (need_update) updateCoKfsAndWeights();
}

void Frame::updateCoKfsAndWeights() {
  LOG(INFO) << cv::format("update_co_kfs_and_weights(for frame: %d)", id_);
  lock_g lock(co_mut_);
  // Filter out those keyframes that the number of shared map points below
  // certain threshold.
  // multimap structure is internally sorted.
  multimap<int, Frame::Ptr> co_weight_kfs;
  for (auto it = co_kf_weights_.cbegin(), it_end = co_kf_weights_.cend();
       it != it_end; ++it) {
    if (it->second <= Config::co_kf_weight_thresh()) continue;
    co_weight_kfs.insert({it->second, it->first});
    // FIXME Need to enable shared_from_this() ?
    it->first->addConnection(shared_from_this(), it->second);
  }

  // Update covisible informations.
  co_kfs_.clear();
  co_weights_.clear();
  for (auto it = co_weight_kfs.cbegin(), it_end = co_weight_kfs.cend();
       it != it_end; ++it) {
    co_kfs_.push_front(it->second);
    co_weights_.push_front(it->first);
  }
}

void Frame::updateCoInfo() {
  LOG(INFO) << cv::format("update_co_info(for frame: %d)", id_);
  // Covisible keyframe voter with the key being the covisible keyframe and the
  // weights being the number of shared map points.
  unordered_map<Frame::Ptr, int> co_kf_weights;
  // co_kf_weights.reserve(?). //! Don't know how much capacity to be reserved.
  //! Map does not have a reserve method whilst unordered_map has.

  // Obtain all covisible keyframes (i.e. ones sharing at least one map point).
  for (const Feature::Ptr& feat_ : feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat_);
    if (!point) continue;
    for (const Feature::Ptr& feat : point->getObservations()) {
      const Frame::Ptr& kf = feat_utils::getKeyframe(feat);
      if (!kf || kf->id_ == id_) continue;  // Self of course is excluded.
      co_kf_weights[kf]++;
    }
  }
  if (co_kf_weights.empty()) return;
  {
    lock_g lock(co_mut_);
    co_kf_weights_ = co_kf_weights;
  }
  updateCoKfsAndWeights();
  LOG(INFO) << "Updated covisible info for keyframe " << id_;
  LOG(INFO) << cv::format("Keyframe %d now has %lu covisible keyframes.", id_,
                          co_kf_weights_.size());
}

double Frame::computeSceneMedianDepth() {
  vector<double> depths;
  depths.reserve(feats_.size());
  for (const Feature::Ptr& feat : feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point) continue;
    const double depth = cam_->world2camera(point->pos()).z();
    depths.push_back(depth);
  }
  if (depths.empty()) return 0.;
  return math_utils::get_median(depths);
}

bool Frame::isObservable(const sptr<MapPoint>& point, const int level) const {
  // Test 1: has positive depth.
  const Vec3& p_w = point->pos();
  const Vec3 p_c = cam_->world2camera(p_w);
  if (p_c(2) < 0.) return false;
  // Test 2: does not go out of image boundary.
  const Vec2 repr_pt = cam_->camera2pixel(p_c);
  if (!(repr_pt.x() >= x_min_ && repr_pt.x() <= x_max_ &&
        repr_pt.y() >= y_min_ && repr_pt.y() <= y_max_))
    return false;
  // FIXME Is the logic right?
  // Test 3: the image pyramid level of the feature is consistent with median
  // scale at which the point is observed. (i.e. within +- 1).
  if (!(level >= point->median_view_scale_ - 1 &&
        level <= point->median_view_scale_ + 1))
    return false;
  // Test 4: viewing direction is consistent with mean viewing direction.
  const double dist = cam_->getDistToCenter(p_w);
  const double cos_view_dir =
      (p_w - cam_->getCamCenter()).dot(point->median_view_dir_) / dist;
  if (cos_view_dir < std::cos(math_utils::degree2radian(60.))) return false;

  // Store the computed result to be used in searching.
  point->repr_x_ = repr_pt.x();
  point->repr_y_ = repr_pt.y();
  point->level_ = level;
  point->cos_view_dir_ = cos_view_dir;
  return true;
}

void Frame::erase() {
  if (id_ == 0) return;  // The first frame is the datum which cannot be erased.

  lock_g lock(co_mut_);
  // Erase covisibility connections.
  auto it = co_kf_weights_.cbegin(), it_end = co_kf_weights_.cend();
  for (; it != it_end; ++it) it->first->deleteConnection(shared_from_this());

  // Erase related observations.
  for (const Feature::Ptr& feat : feats_) {
    const MapPoint::Ptr& point = feat_utils::getPoint(feat);
    if (!point) continue;
    point->eraseObservation(feat);
  }

  // Erase links with map.
  // mpMap->EraseKeyFrame(this);
  // mpKeyFrameDB->erase(this);
}

namespace frame_utils {

void undistortKeypoints(const Mat33& K, const Vec4& dist_coeffs,
                        std::vector<cv::KeyPoint>& kpts) {
  const int num_kpts = kpts.size();
  cv::Mat kpts_mat(num_kpts, 2, CV_64F);
  for (int i = 0; i < num_kpts; ++i) {
    kpts_mat.at<double>(i, 0) = kpts[i].pt.x;
    kpts_mat.at<double>(i, 1) = kpts[i].pt.y;
  }
  cv::Mat K_, dist_coeffs_;
  cv::eigen2cv(K, K_);
  cv::eigen2cv(dist_coeffs, dist_coeffs_);
  cv::undistortPoints(kpts_mat, kpts_mat, K_, dist_coeffs_, cv::Mat{}, K_);
  for (int i = 0; i < num_kpts; ++i) {
    kpts[i].pt.x = kpts_mat.at<double>(i, 0);
    kpts[i].pt.y = kpts_mat.at<double>(i, 1);
  }
}

void computeImageBounds(const cv::Mat& img, const Mat33& K,
                        const Vec4& dist_coeffs, cv::Mat& corners) {
  corners = cv::Mat(4, 2, CV_32F);
  cv::Mat K_, dist_coeffs_;
  cv::eigen2cv(K, K_);
  cv::eigen2cv(dist_coeffs, dist_coeffs_);
  corners.at<float>(0, 0) = 0.0;
  corners.at<float>(0, 1) = 0.0;
  corners.at<float>(1, 0) = img.cols;
  corners.at<float>(1, 1) = 0.0;
  corners.at<float>(2, 0) = 0.0;
  corners.at<float>(2, 1) = img.rows;
  corners.at<float>(3, 0) = img.cols;
  corners.at<float>(3, 1) = img.rows;
  cv::undistortPoints(corners, corners, K_, dist_coeffs_, cv::Mat{}, K_);
}

}  // namespace frame_utils
}  // namespace mono_slam