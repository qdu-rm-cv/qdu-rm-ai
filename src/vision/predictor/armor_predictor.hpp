#pragma once

#include <chrono>
#include <vector>

// #include "buff_predictor.hpp"
#include "armor.hpp"
#include "armor_detector.hpp"
#include "common.hpp"
#include "kalman.hpp"
#include "predictor.hpp"

struct ArmorPredictParam {
  double a;
  double b;
};

class ArmorPredictor : public Predictor<Armor, ArmorPredictParam, Kalman> {
 private:
  Armor armor_;
  std::chrono::milliseconds duration_direction_, duration_predict_;

  void MatchArmor();

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

 public:
  ArmorPredictor();
  ArmorPredictor(const std::string &param);
  ~ArmorPredictor();

  void SetArmor(const Armor &armor);

  const tbb::concurrent_vector<Armor> &Predict();

  void VisualizePrediction(const cv::Mat &output, int add_lable);
};
