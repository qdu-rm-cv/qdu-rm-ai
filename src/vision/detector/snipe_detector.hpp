#pragma once

#include "armor.hpp"
#include "common.hpp"
#include "detector.hpp"
#include "opencv2/gapi.hpp"

struct SnipeDetectorParam {
  int a;
  int b;
};

class SnipeDetector : public Detector<Armor, SnipeDetectorParam> {
 private:
  game::Team enemy_team_;
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  component::Timer duration_armors_;
  cv::gapi::wip::draw::Prims prims_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindArmor(const cv::Mat &frame);

 public:
  SnipeDetector();
  SnipeDetector(const std::string &params_path, game::Team enemy_team);
  ~SnipeDetector();

  void SetEnemyTeam(game::Team enemy_team);
  const tbb::concurrent_vector<Armor> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
