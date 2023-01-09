#pragma once

#include "armor_classifier.hpp"
#include "armor_detector.hpp"
#include "armor_predictor.hpp"
#include "buff_detector.hpp"
#include "buff_predictor.hpp"
#include "common.hpp"
#include "snipe_detector.hpp"

class AimAssitant {
 private:
  ArmorDetector a_detector_;
  ArmorPredictor a_predictor_;
  BuffDetector b_detector_;
  BuffPredictor b_predictor_;
  SnipeDetector s_detector_;
  ArmorClassifier classifier_;

  tbb::concurrent_vector<Armor> armors_;
  component::AimMethod method_ = component::AimMethod::kUNKNOWN;
  game::Arm arm_ = game::Arm::kUNKNOWN;

  void Sort(const cv::Mat& frame);

 public:
  AimAssitant();
  explicit AimAssitant(game::Arm arm);
  ~AimAssitant();

  void LoadParams(const std::string& armor_param, const std::string& buff_param,
                  const std::string& snipe_param,
                  const std::string& armor_pre_param,
                  const std::string& buff_pre_param);
  void SetClassiferParam(const std::string model_path,
                         const std::string lable_path,
                         const cv::Size& input_size);
  void SetEnemyTeam(game::Team enemy_team);
  void SetRFID(game::RFID rfid);
  void SetArm(game::Arm arm);
  void SetRace(game::Race race);
  void SetTime(double time);

  component::AimMethod GetMethod();

  const tbb::concurrent_vector<Armor>& Aim(const cv::Mat& frame);
  void VisualizeResult(const cv::Mat& frame, int add_label = 1);
};
