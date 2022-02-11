#pragma once

#include <chrono>

#include "armor.hpp"
#include "armor_classifier.hpp"
#include "armor_param.hpp"
#include "common.hpp"
#include "detector.hpp"
#include "light_bar.hpp"

class ArmorDetector : public Detector<Armor, ArmorDetectorParam<double>> {
 private:
  game::Team enemy_team_;
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  tbb::concurrent_vector<LightBar> lightbars_;

  std::chrono::milliseconds duration_bars_, duration_armors_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindLightBars(const cv::Mat &frame);
  void MatchLightBars();

  void VisualizeLightBar(const cv::Mat &output, bool add_lable);
  void VisualizeArmor(const cv::Mat &output, bool add_lable);

 public:
  ArmorDetector();
  ArmorDetector(const std::string &params_path, game::Team enemy_team);
  ~ArmorDetector();

  void SetEnemyTeam(game::Team enemy_team);

  const tbb::concurrent_vector<Armor> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
