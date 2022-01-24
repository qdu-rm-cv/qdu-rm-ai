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

  component::AimMethod method_;
  game::Model model_;

 public:
  AimAssitant();
  AimAssitant(game::Model model);
  ~AimAssitant();

  void Init(const std::string& armor_param, const std::string& buff_param,
            const std::string& snipe_param, game::Team enemy_team);
  void SetRFID(game::RFID rfid);

  const tbb::concurrent_vector<Armor>& Detect(const cv::Mat& frame);

  void VisualizeResult(const cv::Mat& frame, int add_label = 1);
};