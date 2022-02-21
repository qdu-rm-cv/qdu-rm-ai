#include "kalman.hpp"

#include <cmath>

#include "spdlog/spdlog.h"

namespace {

const unsigned int kSCALIONGFACTOR = 10;
const unsigned int kWIDTH = 640;
const unsigned int kHEIGHT = 480;

const cv::Point2d kSIZE2(kWIDTH / kSCALIONGFACTOR, kHEIGHT / kSCALIONGFACTOR);
const cv::Point3d kSIZE3(kWIDTH / kSCALIONGFACTOR, kHEIGHT / kSCALIONGFACTOR,
                         std::sqrt(kWIDTH* kHEIGHT) / kSCALIONGFACTOR);

}  // namespace

void Kalman::InnerInit(int states = 4, int measurements = 2) {
  states_ = states;
  measurements_ = measurements;
  error_frame_ = 0;

  cv::Mat empty_matx(cv::Mat_<double>::zeros(2, 1));
  cur_predict_matx_ = empty_matx.clone();
  last_predict_matx_ = empty_matx.clone();
  cur_measure_matx_ = empty_matx.clone();
  last_predict_matx_ = empty_matx.clone();
  empty_matx.release();

  kalman_filter_.init(states, measurements, 1, CV_64F);

  //* A 状态转移矩阵
  cv::Mat temp = cv::Mat_<double>::eye(states, states);
  for (int i = 0; i < states / 2; i++) temp.at<double>(i, i + states / 2) = 1;
  kalman_filter_.transitionMatrix = temp;

  //* B 系统测量矩阵
  kalman_filter_.controlMatrix.release();

  //* H 测量矩阵
  kalman_filter_.measurementMatrix =
      cv::Mat_<double>::eye(measurements, states);

  //* R 测量噪声方差矩阵
  kalman_filter_.measurementNoiseCov =
      cv::Mat_<double>::eye(measurements, measurements);

  //* Q 系统噪声方差矩阵
  // cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1e-5));
  kalman_filter_.processNoiseCov = cv::Mat_<double>::eye(states, states) * 0.03;

  //* P 后验错误估计协方差矩阵
  /* cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(1)); */

  // std::cout << "A:\n" << kalman_filter_.transitionMatrix << "\n";
  // std::cout << "B:\n" << kalman_filter_.controlMatrix << "\n";
  // std::cout << "H:\n" << kalman_filter_.measurementMatrix << "\n";
  // std::cout << "Q:\n" << kalman_filter_.processNoiseCov << "\n";
  // std::cout << "R:\n" << kalman_filter_.measurementNoiseCov << "\n";

  SPDLOG_WARN("A:{}, B:{}, H:{}, Q:{}, R:{}",
              kalman_filter_.transitionMatrix.type(),
              kalman_filter_.controlMatrix.type(),
              kalman_filter_.measurementMatrix.type(),
              kalman_filter_.processNoiseCov.type(),
              kalman_filter_.measurementNoiseCov.type());
}

Kalman::Kalman() {
  InnerInit(4, 2);
  SPDLOG_TRACE("Constructed.");
}

Kalman::Kalman(int states, int measurements) {
  InnerInit(states, measurements);
  SPDLOG_TRACE("Constructed.");
}

Kalman::~Kalman() { SPDLOG_TRACE("Destructed."); }

void Kalman::Init(const std::vector<double>& vec) {
  if (method_ == Method::kUNKNOWN) method_ = Method::kKF;
  SPDLOG_WARN("{}, {}", vec[0], vec[1]);
  InnerInit(static_cast<int>(vec[0]), static_cast<int>(vec[1]));
}

const cv::Point2d Kalman::Predict(const cv::Point2d& measurements_point) {
  last_predict_matx_ = cur_predict_matx_;
  last_measure_matx_ = cur_measure_matx_;

  cv::Point2d diff = cv::Point2d(last_measure_matx_.at<double>(0, 0),
                                 last_measure_matx_.at<double>(0, 1)) -
                     measurements_point;

  if (abs(diff.x) > kSIZE2.x || abs(diff.y) > kSIZE2.y)
    error_frame_ += 1;
  else if (measurements_point == cv::Point2d(0., 0.))
    error_frame_ += 1;

  if (error_frame_ > 0 && error_frame_ < 5)
    cur_measure_matx_ = last_predict_matx_.rowRange(0, 2);
  else {
    cv::Mat measurements = cv::Mat_<double>::zeros(2, 1);
    measurements.at<double>(0, 0) = measurements_point.x;
    measurements.at<double>(0, 1) = measurements_point.y;
    cur_measure_matx_ = measurements;
  }

  SPDLOG_WARN("Error frames count : {}", error_frame_);
  cur_predict_matx_ = kalman_filter_.correct(cur_measure_matx_);
  cur_predict_matx_ = kalman_filter_.predict();
  SPDLOG_WARN("Predicted.");
  return cv::Point2d(cur_predict_matx_.at<double>(0, 0),
                     cur_predict_matx_.at<double>(0, 1));
}

const cv::Point3d Kalman::Predict(const cv::Point3d& measurements_point) {
  last_predict_matx_ = cur_predict_matx_;
  last_measure_matx_ = cur_measure_matx_;

  cv::Point3d diff = cv::Point3d(last_measure_matx_.at<double>(0, 0),
                                 last_measure_matx_.at<double>(0, 1),
                                 last_measure_matx_.at<double>(0, 2)) -
                     measurements_point;

  if (abs(diff.x) > kSIZE3.x || abs(diff.y) > kSIZE3.y ||
      abs(diff.z) > kSIZE3.z)
    error_frame_ += 1;
  else if (measurements_point == cv::Point3d(0., 0., 0.))
    error_frame_ += 1;

  if (error_frame_ > 0 && error_frame_ < 5)
    cur_measure_matx_ = last_predict_matx_.rowRange(0, 3);
  else {
    cv::Mat measurements = cv::Mat_<double>::zeros(3, 1);
    measurements.at<double>(0, 0) = measurements_point.x;
    measurements.at<double>(0, 1) = measurements_point.y;
    measurements.at<double>(0, 2) = measurements_point.z;
    cur_measure_matx_ = measurements;
  }

  SPDLOG_WARN("Error frames count : {}", error_frame_);
  cur_predict_matx_ = kalman_filter_.correct(cur_measure_matx_);
  cur_predict_matx_ = kalman_filter_.predict();
  SPDLOG_WARN("Predicted.");
  return cv::Point3d(cur_predict_matx_.at<double>(0, 0),
                     cur_predict_matx_.at<double>(0, 1),
                     cur_predict_matx_.at<double>(0, 2));
}

const cv::Mat& Kalman::Predict(const cv::Mat& measurements) {
  last_predict_matx_ = cur_predict_matx_;
  last_measure_matx_ = cur_measure_matx_;

  std::vector<double> measure_value(measurements_);
  std::vector<double> last_measure_value(measurements_);
  double product = measure_value.size() > 1 ? 1 : measure_value.back();
  const unsigned int edge = std::min(kSIZE2.x, kSIZE2.y);

  for (std::size_t i = 0; i < measure_value.size(); i++) {
    measure_value[i] = measurements.at<double>(0, i);
    last_measure_value[i] = last_measure_matx_.at<double>(0, i);
    product *= measure_value[i];
  }

  if (product == 0)
    error_frame_ += 1;
  else
    for (std::size_t i = 0; i < measure_value.size(); i++)
      if (abs(measure_value[i] - last_measure_value[i]) > edge) {
        error_frame_ += 1;
        break;
      }

  if (error_frame_ > 0 && error_frame_ < 5)
    cur_measure_matx_ = last_predict_matx_.rowRange(0, states_);
  else
    cur_measure_matx_ = measurements;

  SPDLOG_WARN("Error frames count : {}", error_frame_);
  cur_predict_matx_ = kalman_filter_.correct(cur_measure_matx_);
  cur_predict_matx_ = kalman_filter_.predict();
  SPDLOG_WARN("Predicted.");
  return cur_predict_matx_;
}
