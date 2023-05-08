#pragma once

#include <vector>

#include "common.hpp"
#include "light_bar.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"

enum ArmorType { SMALL = 0, LARGE = 1 };
class Armor : public ImageObject, public PhysicObject {
 private:
  game::Model model_ = game::Model::kUNKNOWN;
  component::Euler aiming_euler_;
  cv::RotatedRect rect_;
  cv::RotatedRect FormRect(const LightBar &left_bar, const LightBar &right_bar);
  void Init();

 public:
  ArmorType armor_type_;
  double confidence_;
  std::string number_;
  std::string classfication_result_;

  Armor();
  Armor(const LightBar &left_bar, const LightBar &right_bar);
  explicit Armor(const cv::RotatedRect &rect);
  ~Armor();

  game::Model GetModel() const;
  void SetModel(game::Model model);
  const cv::RotatedRect GetRect() const;
  cv::Mat Face(const cv::Mat &frame);
  double GetArea();
  component::Euler GetAimEuler() const;
  void SetAimEuler(const component::Euler &elur);
  ArmorType GetArmorType();
};
