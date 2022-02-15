#pragma once

#include "common.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"

class OreCube : public ImageObject, public PhysicObject {
 private:
  component::Euler self_euler_;
  cv::Point3f center_;
  float radius_;

  void Init(const cv::RotatedRect& rect);

 public:
  OreCube();
  OreCube(const cv::Point2f& center, float radius);
  OreCube(const cv::RotatedRect& rect);
  ~OreCube();

  component::Euler GetSelfEuler();
  const cv::Point3f& GetPhysicCenter();
  const float& GetRadius();
};
