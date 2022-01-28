#pragma once

#include "armor_detector.hpp"
#include "buff_detector.hpp"
#include "common.hpp"
#include "snipe_detector.hpp"

class AimAssitant {
 private:
  BuffDetector b_detector_;
  ArmorDetector a_detector_;
  SnipeDetector s_detector_;

  component::AimMethod method_ = component::AimMethod::kUNKNOWN;
  game::Arm arm_ = game::Arm::kUNKNOWN;

 public:
  AimAssitant();
  AimAssitant(game::Arm arm);
  ~AimAssitant();

  void LoadParams(const std::string& armor_param, const std::string& buff_param,
                  const std::string& snipe_param);
  void SetEnemyTeam(game::Team enemy_team);
  void SetRFID(game::RFID rfid);
  void SetArm(game::Arm arm);

  const tbb::concurrent_vector<Armor>& Detect(const cv::Mat& frame);

  void VisualizeResult(const cv::Mat& frame, int add_label = 1);
};