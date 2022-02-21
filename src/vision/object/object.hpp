#pragma once

#include <vector>

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace draw {

const auto kCV_FONT = cv::FONT_HERSHEY_SIMPLEX;

const cv::Scalar kBLUE(255., 0., 0.);
const cv::Scalar kGREEN(0., 255., 0.);
const cv::Scalar kRED(0., 0., 255.);
const cv::Scalar kYELLOW(0., 255., 255.);
const cv::Scalar kBLACK(0., 0., 0.);

void VisualizeLabel(const cv::Mat &output, const std::string &label,
                    int level = 1, const cv::Scalar &color = kGREEN);

}  // namespace draw

class ImageObject {
 public:
  std::vector<cv::Point2f> image_vertices_;
  cv::Point2f image_center_;
  cv::Size face_size_;
  cv::Mat trans_;
  float image_angle_;
  double image_ratio_;

  const cv::Point2f &ImageCenter() const;

  std::vector<cv::Point2f> ImageVertices() const;

  double ImageAngle() const;

  double ImageAspectRatio() const;

  cv::Mat ImageFace(const cv::Mat &frame) const;

  void VisualizeObject(const cv::Mat &output, bool add_lable,
                       const cv::Scalar color = draw::kGREEN,
                       cv::MarkerTypes type = cv::MarkerTypes::MARKER_DIAMOND);
};

class PhysicObject {
 public:
  cv::Mat rot_vec_, rot_mat_, trans_vec_, physic_vertices_;

  const cv::Mat &GetRotVec() const;
  void SetRotVec(const cv::Mat &rot_vec);

  const cv::Mat &GetRotMat() const;
  void SetRotMat(const cv::Mat &rot_mat);

  const cv::Mat &GetTransVec() const;
  void SetTransVec(const cv::Mat &trans_vec);

  cv::Vec3d RotationAxis() const;
  const cv::Mat PhysicVertices() const;
};
