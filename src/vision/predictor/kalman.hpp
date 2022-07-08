#pragma once
#include "filter.hpp"

class Kalman : public Filter {
 private:
  cv::KalmanFilter kalman_filter_;
  cv::Mat cur_predict_matx_, last_predict_matx_;
  cv::Mat cur_measure_matx_, last_measure_matx_;
  unsigned int error_frame_, start_frame_;
  std::vector<cv::Point2d> coords_;

  void InnerInit(int states, int measurements);

 public:
  Kalman();
  Kalman(int states, int measurements);
  ~Kalman();

  void Init(const std::vector<double>& vec);
  bool Config(const cv::Point2d& measurements_point);

  const cv::Point2d Predict(const cv::Point2d& measurements_point);
  const cv::Point3d Predict(const cv::Point3d& measurements_point);
  const cv::Mat& Predict(const cv::Mat& measurements);
};
