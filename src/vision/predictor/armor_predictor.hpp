#pragma once

#include <chrono>
#include <vector>

#include "armor.hpp"
#include "armor_detector.hpp"
#include "common.hpp"
#include "kalman.hpp"
#include "opencv2/gapi.hpp"
#include "predictor.hpp"
#include "timer.hpp"

struct ArmorPredictParam {
  double a;
  double b;
};

class ArmorPredictor : public Predictor<Armor, ArmorPredictParam, Kalman> {
 private:
  Armor armor_;
  tbb::concurrent_vector<Armor> armors_;
  component::Timer duration_direction_, duration_predict_;
  cv::gapi::wip::draw::Prims prims_;

  void MatchArmor();

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

 public:
  ArmorPredictor();
  ArmorPredictor(const std::string &param);
  ~ArmorPredictor();

  void SetArmor(const Armor &armor);
  void SetArmors(const tbb::concurrent_vector<Armor> &armors);

  const tbb::concurrent_vector<Armor> &Predict();

  void VisualizePrediction(const cv::Mat &output, int add_lable);
};
