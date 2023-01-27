#pragma once

#include "armor_detector.hpp"
#include "buff.hpp"
#include "buff_param.hpp"
#include "detector.hpp"
#include "opencv2/opencv.hpp"

class BuffDetector : public Detector<Buff, BuffDetectorParam<double>> {
 private:
  Buff buff_;
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  cv::RotatedRect hammer_;
  game::Team team_ = game::Team::kUNKNOWN;

  component::Timer duration_armors_, duration_buff_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void MatchBuff(const cv::Mat &frame);

  void VisualizeArmors(const cv::Mat &output, bool add_lable);

 public:
  BuffDetector();
  BuffDetector(const std::string &param_path, game::Team enemy_team);
  ~BuffDetector();

  void SetTeam(game::Team enemy_team);

  const tbb::concurrent_vector<Buff> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &frame, int verbose);
};
