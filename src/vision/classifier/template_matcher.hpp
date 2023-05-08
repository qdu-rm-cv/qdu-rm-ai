#pragma once

#include <vector>

#include "armor.hpp"
#include "common.hpp"
#include "opencv2/opencv.hpp"

class TemplaterMatcher {
 public:
  int GetArmorId(cv::Mat &armor_face);
  double MatchTemplate(cv::Mat &img, cv::Mat &temp);
};
