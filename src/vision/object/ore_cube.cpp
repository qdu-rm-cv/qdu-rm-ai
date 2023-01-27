#include "ore_cube.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kSIDE = 210.;

std::vector<cv::Point2f> k2D_ORECUBE{
    cv::Point2f(0, kSIDE),
    cv::Point2f(0, 0),
    cv::Point2f(kSIDE, 0),
    cv::Point2f(kSIDE, kSIDE),
};

/* clang-format off */
const cv::Matx43f k3D_ORECUBE(
  -kSIDE / 2, -kSIDE / 2, kSIDE,
  kSIDE / 2, -kSIDE / 2, 0.,
  kSIDE / 2, kSIDE / 2, 0.,
  -kSIDE / 2, kSIDE / 2, kSIDE);
/* clang-format on */

}  // namespace

void OreCube::Init(const cv::RotatedRect& rect) {
  image_angle_ = rect.angle;
  image_center_ = rect.center;
  image_ratio_ = rect.size.aspectRatio();
  image_vertices_.resize(4);
  rect.points(image_vertices_.data());
  trans_ = cv::getPerspectiveTransform(ImageVertices(), k2D_ORECUBE);
  face_size_ = cv::Size(kSIDE, kSIDE);
}

OreCube::OreCube() { SPDLOG_TRACE("Constructed."); }

OreCube::OreCube(const cv::Point2f& center, float radius) {
  radius_ = radius;
  double side = radius * 2 / std::sqrt(3);
  cv::RotatedRect rect(center, cv::Size(side, side), 0);
  Init(rect);
  SPDLOG_TRACE("Constructed.");
}

OreCube::OreCube(const cv::RotatedRect& rect) {
  radius_ = 0.5 * std::sqrt(rect.size.area() +
                            std::pow(cv::norm(cv::Point2f(rect.size)), 2));
  Init(rect);
  SPDLOG_TRACE("Constructed.");
}

OreCube::~OreCube() { SPDLOG_TRACE("Destructed."); }

component::Euler OreCube::GetSelfEuler() { return self_euler_; }

const cv::Point3f& OreCube::GetPhysicCenter() { return center_; }

const float& OreCube::GetRadius() { return radius_; }
