#pragma once

#include <vector>

#include "armor.hpp"
#include "common.hpp"
#include "tbb/concurrent_vector.h"

class Buff {
 private:
  cv::Point2f center_;
  tbb::concurrent_vector<Armor> armors_;
  Armor target_;

 public:
  Buff();
  Buff(game::Team team);
  Buff(const cv::Point2f &center, const tbb::concurrent_vector<Armor> &armors,
       const Armor &target);
  ~Buff();

  const tbb::concurrent_vector<Armor> &GetArmors() const;
  void SetArmors(const tbb::concurrent_vector<Armor> &armors);

  const cv::Point2f &GetCenter() const;
  void SetCenter(const cv::Point2f &center);

  const Armor &GetTarget() const;
  void SetTarget(const Armor &target);
};