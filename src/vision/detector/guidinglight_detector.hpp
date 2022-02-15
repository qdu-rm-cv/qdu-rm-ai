#pragma once

#include <chrono>

#include "common.hpp"
#include "detector.hpp"
#include "guiding_light.hpp"

class GuidingLightDetector
    : public Detector<GuidingLight, cv::SimpleBlobDetector::Params> {
 private:
  cv::Ptr<cv::SimpleBlobDetector> detector_;

  std::vector<cv::KeyPoint> key_points_;
  std::chrono::milliseconds duration_lights_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  void FindGuidingLight(const cv::Mat &frame);

 public:
  GuidingLightDetector();
  GuidingLightDetector(const std::string &params_path);
  ~GuidingLightDetector();

  void ResetByParam(cv::SimpleBlobDetector::Params param);

  const tbb::concurrent_vector<GuidingLight> &Detect(const cv::Mat &frame);
  void VisualizeResult(const cv::Mat &output, int verbose = 1);
};
