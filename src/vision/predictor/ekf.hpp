#pragma once

#include <Eigen/Dense>

#include "ceres/jet.h"
#include "filter.hpp"
#include "opencv2/core/eigen.hpp"

class EKF : public Filter {
 public:
  using EMatx55d = Eigen::Matrix<double, 5, 5>;
  using EMatx35d = Eigen::Matrix<double, 3, 5>;
  using EMatx53d = Eigen::Matrix<double, 5, 3>;
  using EMatx33d = Eigen::Matrix<double, 3, 3>;
  using EMatx51d = Eigen::Matrix<double, 5, 1>;
  using EMatx31d = Eigen::Matrix<double, 3, 1>;

  using Matx33d = cv::Matx33d;
  using Matx35d = cv::Matx<double, 3, 5>;
  using Matx53d = cv::Matx<double, 5, 3>;
  using Matx55d = cv::Matx<double, 5, 5>;
  using Matx31d = cv::Matx<double, 3, 1>;
  using Matx51d = cv::Matx<double, 5, 1>;

 private:
  cv::Mat Xe;  // Matx51d state_pre_;             /* 估计状态变量 */
  cv::Mat Xp;  // Matx51d state_next_;            /* 预测状态变量 */
  // Matx55d F;   // Matx55d predict_mat_;           /* 预测雅克比 */
  // Matx35d H;   // Matx35d measurement_mat_;       /* 观测雅克比 */
  // Matx55d P;   // Matx55d state_cov_;             /* 状态协方差 */
  // Matx55d Q;   // Matx55d process_noi_cov_mat_;   /* 预测过程协方差 */
  // Matx33d R;   // Matx33d measure_noi_cov_mat_;   /* 观测过程协方差 */
  // Matx53d K;   // Matx35d kalman_gain_;           /* 卡尔曼增益 */
  // Matx31d Yp;  // Matx31d predict_obs_;           /* 预测观测量 */

  EMatx51d EXe;
  EMatx51d EXp;
  EMatx55d EF;
  EMatx35d EH;
  EMatx55d EP;
  EMatx55d EQ;
  EMatx33d ER;
  EMatx53d EK;
  EMatx31d EYp;

  double delta_t_;

  void InnerInit(const Matx51d& Xe);

 public:
  EKF();
  explicit EKF(const Matx51d& Xe);
  ~EKF();

  void Init(const std::vector<double>& vec);

  const cv::Mat& Predict(const cv::Mat& measurements);
  const cv::Mat& Update(const cv::Mat& upgrade);
};
