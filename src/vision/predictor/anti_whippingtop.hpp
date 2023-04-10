#pragma once

#include <chrono>
#include <iostream>

#include "antitop_param.hpp"
#include "armor_detector.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"
#include "predictor.hpp"
#include "timer.hpp"
#include "kalman.hpp"

// param start

struct WhippingTopParam {
  double center_height_diff_low;
  double center_height_diff_high;
  double center_width_diff_low;
  double center_width_diff_high;

  int detector_param_th;       // 持续识别系数
  int whipping_top_param_th;   // 陀螺系数
  int missing_frame_param_th;  // 掉帧系数
};

// param whipping_top end

enum class Move {
  kLeft,
  kRight,
  kStatic
};

class AntiWhippingTop : public Predictor
  <Armor, AntiWhippingTopParam<double>, Kalman> {
 private:
  game::Team enemy_team_;
  game::Model model_ = game::Model::kUNKNOWN;
  component::Direction direction_ = component::Direction::kUNKNOWN;
  Move move_ = Move::kStatic;
  bool is_pre_loaded_ = 0, is_whipping_ = 0;
  cv::Point2f target_center_;
  Armor pre_armor_, now_armor_, target_armor_;
  tbb::concurrent_vector<Armor> detects_;
  component::Timer duration_iswhipping_, duration_strategy_, duration_predict_;
  int antitop_no_increase = 0;

 public:
  int missing_frame_param_ = 0;
  int whipping_top_param_ = 0;
  int detector_param_ = 0;

 private:
  void GetRotation();
  void InitPreArmor(const Armor armor_);
  void InitNowArmor(const Armor armor_);
  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);
  const Armor& GetTargetArmor(cv::RotatedRect pre_rect,
    cv::RotatedRect now_rect);
  bool ArmorClassify(int model);
  void AntiStrategy();
  void EndWhipping();
  bool IsWhipping(const tbb::concurrent_vector<Armor> targets);

 public:
  AntiWhippingTop(const std::string params_path, game::Team enemy_team,
    const tbb::concurrent_vector<Armor> detects);
  AntiWhippingTop();
  ~AntiWhippingTop();
  void LoadParams(const std::string &path);
  const tbb::concurrent_vector<Armor> &Predict();
  void VisualizePrediction(const cv::Mat &output, int add_label);
  void SetEnemyTeam(game::Team enemy_team);
  // 与ArmorDetect的接口
  void SetDetects(const tbb::concurrent_vector<Armor> detects);
  bool GetIsWhipping();
};
