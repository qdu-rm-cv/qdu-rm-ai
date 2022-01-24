#pragma once

#include <chrono>

#include "armor.hpp"
#include "common.hpp"
#include "detector.hpp"

struct SnipeDetectorParam {
  int a;
  int b;
};

class SnipeDetector : public Detector<Armor, SnipeDetectorParam> {
 private:
  game::Team enemy_team_;
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  std::chrono::milliseconds duration_bars_, duration_armors_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindArmor(const cv::Mat &frame);

  void VisualizeArmor(const cv::Mat &output, bool add_lable);

 public:
  SnipeDetector();
  SnipeDetector(const std::string &params_path, game::Team enemy_team);
  ~SnipeDetector();

  void SetEnemyTeam(game::Team enemy_team);
  const tbb::concurrent_vector<Armor> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
