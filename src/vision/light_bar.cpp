#include "light_bar.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

LightBar::LightBar() { SPDLOG_TRACE("Constructed."); }

LightBar::LightBar(const cv::RotatedRect &rect) {
  rect_ = rect;
  Init();
  SPDLOG_TRACE("Constructed.");
}

LightBar::~LightBar() { SPDLOG_TRACE("Destructed."); }

void LightBar::Init() {
  image_center_ = rect_.center;
  image_angle_ = rect_.angle;

  image_vertices_.resize(4);
  rect_.points(image_vertices_.data());

  if (rect_.size.width > rect_.size.height) {
    rect_.angle -= 90.;
    std::swap(rect_.size.width, rect_.size.height);
  }
  face_size_ = rect_.size;

  if (rect_.angle > 90.) {
    image_angle_ = rect_.angle - 180.;
  } else if (rect_.angle > 270.) {
    image_angle_ = 360. - rect_.angle;
  } else {
    image_angle_ = rect_.angle;
  }

  image_ratio_ = std::max(rect_.size.height, rect_.size.width) /
                 std::min(rect_.size.height, rect_.size.width);

  SPDLOG_DEBUG("Inited.");
}

double LightBar::Area() const { return rect_.size.area(); }

double LightBar::Length() const {
  SPDLOG_DEBUG("rect_.size (h,w): ({}, {})", rect_.size.height,
               rect_.size.width);
  return std::max(rect_.size.height, rect_.size.width);
}
