#include "buff.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

Buff::Buff() : center_(cv::Point2f(0, 0)) { SPDLOG_TRACE("Constructed."); }

Buff::Buff(const cv::Point2f& center, const std::vector<Armor>& armors,
           const Armor& target) {
  SetCenter(center);
  SetArmors(armors);
  SetTarget(target);
  SPDLOG_TRACE("Constructed.");
}

Buff::~Buff() { SPDLOG_TRACE("Destructed."); }

const std::vector<Armor>& Buff::GetArmors() const {
  SPDLOG_DEBUG("armors_: {}", armors_.size());
  return armors_;
}

void Buff::SetArmors(const std::vector<Armor>& armors) { armors_ = armors; }

const cv::Point2f& Buff::GetCenter() const {
  SPDLOG_DEBUG("center_: {}, {}", center_.x, center_.y);
  return center_;
}

void Buff::SetCenter(const cv::Point2f& center) { center_ = center; }

const Armor& Buff::GetTarget() const { return target_; }

void Buff::SetTarget(const Armor& target) { target_ = target; }
