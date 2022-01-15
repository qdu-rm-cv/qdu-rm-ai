#pragma once
#include "filter.hpp"

class Kalman : public Filter {
 private:
  cv::KalmanFilter kalman_filter_;
  void Init(int states, int measurements);
  cv::Mat cur_predict_matx_, last_predict_matx_;
  cv::Mat cur_measure_matx_, last_measure_matx_;
  unsigned int error_frame_;

 public:
  Kalman();
  Kalman(int states, int measurements);
  ~Kalman();
  const cv::Point2d Predict(const cv::Point2d& measurements_point,
                            const cv::Mat& frame);
  const cv::Mat& Predict(const cv::Mat& measurements, const cv::Mat& frame);
};
