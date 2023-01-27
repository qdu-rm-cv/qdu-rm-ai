#pragma once

#include "detector.hpp"
#include "ore_cube.hpp"

struct OreCubeDetectorParam {
  double hue_low_th;
  double hue_high_th;
  double saturation_low_th;
  double saturation_high_th;
  double value_low_th;
  double value_high_th;
  int binary_th;
  double area_low_th;
  double area_high_th;
};

class OreCubeDetector : public Detector<OreCube, OreCubeDetectorParam> {
 private:
  std::vector<std::vector<cv::Point>> contours_, contours_poly_;
  component::Timer duration_cube_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindOreCube(const cv::Mat &frame);

 public:
  OreCubeDetector();
  explicit OreCubeDetector(const std::string &params_path);
  ~OreCubeDetector();

  const tbb::concurrent_vector<OreCube> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
