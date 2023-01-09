#pragma once

#include <vector>

#include "common.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"
#include "tbb/concurrent_vector.h"
#include "timer.hpp"

template <typename Target, typename Param, typename Filter>
class Predictor {
 private:
  virtual void InitDefaultParams(const std::string &path) = 0;
  virtual bool PrepareParams(const std::string &path) = 0;

 public:
  tbb::concurrent_vector<Target> predicts_;
  Param params_;
  Filter filter_;
  component::Direction direction_ = component::Direction::kUNKNOWN;

  void LoadParams(const std::string &path) {
    if (!PrepareParams(path)) {
      InitDefaultParams(path);
      PrepareParams(path);
      SPDLOG_WARN("Can not find params file. Created and reloaded.");
    }
    SPDLOG_DEBUG("Params loaded.");
  }

  virtual const tbb::concurrent_vector<Target> &Predict() = 0;
  virtual void VisualizePrediction(const cv::Mat &output, int add_lable) = 0;
};
