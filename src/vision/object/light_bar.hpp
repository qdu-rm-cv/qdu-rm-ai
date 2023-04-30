#pragma once

#include <vector>

#include "object.hpp"
#include "opencv2/opencv.hpp"

class LightBar : public ImageObject, public PhysicObject {
 private:
  cv::RotatedRect rect_;

  void Init();

 public:
  LightBar();
  explicit LightBar(const cv::RotatedRect& rect);
  ~LightBar();

  double Area() const;
  double Length() const;
  cv::Rect GetLightBarROI();
};
