#pragma once

#include "armor.hpp"
#include "armor_param.hpp"
#include "detector.hpp"
#include "light_bar.hpp"
#include "timer.hpp"

class ArmorDetector : public Detector<Armor, ArmorDetectorParam<double>> {
 private:
  game::Team enemy_team_;
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  tbb::concurrent_vector<LightBar> lightbars_;

  component::Timer duration_bars_, duration_armors_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindLightBars(const cv::Mat &frame);
  void MatchLightBars(const cv::Mat frame102);

 public:
  ArmorDetector();
  ArmorDetector(const std::string &params_path, game::Team enemy_team);
  ~ArmorDetector();

  void SetEnemyTeam(game::Team enemy_team);

  const tbb::concurrent_vector<Armor> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
