#pragma once
#include <string>

#include "opencv2/opencv.hpp"
#include "param.hpp"
#include "spdlog/spdlog.h"

struct GuidingLightDetectorParam {
  int thresholdStep;
  int minThreshold;
  int maxThreshold;
  std::size_t minRepeatability;
  int minDistBetweenBlobs;

  bool filterByColor;
  unsigned char blobColor;

  bool filterByArea;
  int minArea, maxArea;

  bool filterByCircularity;
  int minCircularity, maxCircularity; /*圆度值*/

  bool filterByInertia;
  int minInertiaRatio, maxInertiaRatio; /*惯性比*/

  bool filterByConvexity;
  int minConvexity, maxConvexity; /*凸度*/
};

class GuidingLightParam
    : public Param<GuidingLightDetectorParam, cv::SimpleBlobDetector::Params> {
 public:
  cv::SimpleBlobDetector::Params TransformToDouble();

  bool Read(const std::string &params_path) override;
  void Write(const std::string &params_path) override;
};
