#pragma once

#include <chrono>

#include "common.hpp"
#include "detector.hpp"
#include "ore_cube.hpp"

struct OreCubeDetectorParam {
  int a;
  int b;
};

class OreCubeDetector : public Detector<OreCube, OreCubeDetectorParam> {
 private:
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  std::chrono::milliseconds duration_bars_, duration_armors_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindOreCube(const cv::Mat &frame);

  void VisualizeOreCube(const cv::Mat &output, bool add_lable);

 public:
  OreCubeDetector();
  OreCubeDetector(const std::string &params_path);
  ~OreCubeDetector();

  const tbb::concurrent_vector<OreCube> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
