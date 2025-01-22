/*
 * @Description: some tools
 * @Autor: Zijie Chen
 * @Date: 2023-12-27 23:43:58
 */

#pragma once

#include <vector>

#include <Eigen/Core>

template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data,
                           D& mean,
                           D& cov_diag,
                           Getter&& getter) {
  size_t len = data.size();
  assert(len > 1);
  // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
  // clang-format on
}

inline bool EstimatePlane(Eigen::Vector4d& pca_result,
                          const std::vector<Eigen::Vector3d>& point,
                          const double threshold = 0.1) {
  if (point.size() < 3) {
    return false;
  }

  Eigen::MatrixXd A(point.size(), 3);
  Eigen::VectorXd b(point.size(), 1);

  A.setZero();
  b.setOnes();
  b *= -1.0;

  for (size_t j = 0; j < point.size(); j++) {
    A(j, 0) = point[j].x();
    A(j, 1) = point[j].y();
    A(j, 2) = point[j].z();
  }

  Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

  double n_norm = normvec.norm();
  pca_result(0) = normvec(0) / n_norm;
  pca_result(1) = normvec(1) / n_norm;
  pca_result(2) = normvec(2) / n_norm;
  pca_result(3) = 1.0 / n_norm;

  for (const auto& p : point) {
    Eigen::Vector4d temp(p.x(), p.y(), p.z(), 1.0);
    if (fabs(pca_result.dot(temp)) > threshold) {
      return false;
    }
  }
  return true;
}
// 输入参数e：当前误差（例如，点云匹配的平方误差）。delta：Cauchy 损失的尺度参数（影响 inlier 和 outlier 的判定阈值）。
// δ 越大，更多误差被认为是 inlier。rho：输出为一个 3 维向量，分别是：rho[0]：损失函数的值 rho[1]：损失函数的一阶导数 
// rho[2]：损失函数的二阶导数 
// 误差小（inlier）：
// 损失值与误差值成正比。
// 即：直接使用误差，不对其修正。
// 误差大（outlier）：
//  ，对大误差进行平滑处理。
// 减少异常值对优化过程的影响。
inline void CauchyLossFunction(const double e,
                               const double delta,
                               Eigen::Vector3d& rho) {
  double dsqr = delta * delta;
  if (e <= dsqr) {  // inlier
    rho[0] = e;
    rho[1] = 1.;
    rho[2] = 0.;
  } else {                   // outlier
    double sqrte = sqrt(e);  // absolut value of the error
    // rho(e)   = 2 * delta * e^(1/2) - delta^2
    rho[0] = 2 * sqrte * delta - dsqr;
    // rho'(e)  = delta / sqrt(e)
    rho[1] = delta / sqrte;
    // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
    rho[2] = -0.5 * rho[1] / e;
  }
}