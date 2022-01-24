#pragma once

#include <vector>

#include "common.hpp"
#include "light_bar.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"

class Armor : public ImageObject, public PhysicObject {
 private:
  game::Model model_ = game::Model::kUNKNOWN;
  component::Euler aiming_euler_;
  cv::RotatedRect rect_;

  cv::RotatedRect FormRect(const LightBar &left_bar, const LightBar &right_bar);
  void Init();

 public:
  Armor();
  Armor(const LightBar &left_bar, const LightBar &right_bar);
  Armor(const cv::RotatedRect &rect);
  ~Armor();

  game::Model GetModel() const;
  void SetModel(game::Model model);

  component::Euler GetAimEuler() const;
  void SetAimEuler(const component::Euler &elur);
};
