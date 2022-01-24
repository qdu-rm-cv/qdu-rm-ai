#pragma once

#include <vector>

#include "common.hpp"
#include "opencv2/opencv.hpp"

enum class Method {
  kUNKNOWN,
  kKF,
  kEKF,
};

class Filter {
 private:
  bool predict_condition_;

 public:
  Method method_ = Method::kUNKNOWN;
  unsigned int measurements_, states_;

  virtual void Init(const std::vector<double>& vec) = 0;

  virtual const cv::Mat& Predict(const cv::Mat& measurements,
                                 const cv::Mat& frame) = 0;
};

// TODO : 看进度和效果引入UKF