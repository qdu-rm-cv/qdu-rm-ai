#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "object.hpp"

class GuidingLight {
 private:
  cv::KeyPoint key_point_;

 public:
  GuidingLight();
  explicit GuidingLight(const cv::KeyPoint &key_point);
  ~GuidingLight();

  const cv::KeyPoint GetKeyPoint() const;
};
