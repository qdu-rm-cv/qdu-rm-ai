#include "guiding_light.hpp"

#include "spdlog/spdlog.h"

GuidingLight::GuidingLight() { SPDLOG_TRACE("Constructed."); }

GuidingLight::GuidingLight(const cv::KeyPoint &key_point) {
  key_point_ = key_point;
}

GuidingLight::~GuidingLight() { SPDLOG_TRACE("Destructed."); }

const cv::KeyPoint GuidingLight::GetKeyPoint() const { return key_point_; }
