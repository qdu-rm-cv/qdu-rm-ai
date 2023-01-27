#pragma once

#include <vector>

#include "common.hpp"
#include "opencv2/opencv.hpp"

class Filter {
 private:
  bool predict_condition_;

 public:
  component::FilterMethod method_ = component::FilterMethod::kUNKNOWN;
  unsigned int measurements_, states_;

  virtual void Init(const std::vector<double>& vec) = 0;

  virtual const cv::Mat& Predict(const cv::Mat& measurements) = 0;
};

// TODO(C.Meng) : 看进度和效果引入UKF
