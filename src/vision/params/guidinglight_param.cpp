#include "guidinglight_param.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

bool GuidingLightParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    param_int.thresholdStep = fs["thresholdStep"];
    param_int.minThreshold = fs["minThreshold"];
    param_int.maxThreshold = fs["maxThreshold"];

    param_int.minRepeatability = int(fs["minRepeatability"]);
    param_int.minDistBetweenBlobs = fs["minDistBetweenBlobs"];

    param_int.filterByColor = int(fs["filterByColor"]) != 0 ? true : false;
    param_int.blobColor = int(fs["blobColor"]);
    param_int.filterByArea = int(fs["filterByArea"]) != 0 ? true : false;
    param_int.minArea = fs["minArea"];
    param_int.maxArea = fs["maxArea"];

    param_int.filterByCircularity =
        int(fs["filterByCircularity"]) != 0 ? true : false;
    param_int.minCircularity = fs["minCircularity"];
    param_int.maxCircularity = fs["maxCircularity"];

    param_int.filterByInertia = int(fs["filterByInertia"]) != 0 ? true : false;
    param_int.minInertiaRatio = fs["minInertiaRatio"];
    param_int.maxInertiaRatio = fs["maxInertiaRatio"];

    param_int.filterByConvexity =
        int(fs["filterByConvexity"]) != 0 ? true : false;
    param_int.minConvexity = fs["minConvexity"];
    param_int.maxConvexity = fs["maxConvexity"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

cv::SimpleBlobDetector::Params GuidingLightParam::transform2Double() {
  cv::SimpleBlobDetector::Params param;
  param.thresholdStep = param_int.thresholdStep / 1.;
  param.minThreshold = param_int.minThreshold / 1.;
  param.maxThreshold = param_int.maxThreshold / 1.;

  param.minRepeatability = param_int.minRepeatability;
  param.minDistBetweenBlobs = param_int.minDistBetweenBlobs / 1.;

  param.filterByColor = param_int.filterByColor;
  param.blobColor = param_int.blobColor;

  param.filterByArea = param_int.filterByArea;
  param.minArea = param_int.minArea / 1.;
  param.maxArea = param_int.maxArea / 1.;

  param.filterByCircularity = param_int.filterByCircularity;
  param.minCircularity = param_int.minCircularity / 20.;
  param.maxCircularity = param_int.maxCircularity / 20.;

  param.filterByInertia = param_int.filterByInertia;
  param.minInertiaRatio = param_int.minInertiaRatio / 20.;
  param.maxInertiaRatio = param_int.maxInertiaRatio / 20.;

  param.filterByConvexity = param_int.filterByConvexity;
  param.minConvexity = param_int.minConvexity / 20.;
  param.maxConvexity = param_int.maxConvexity / 20.;
  return param;
}

void GuidingLightParam::Write(const std::string &params_path) const {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "thresholdStep" << param_int.thresholdStep;
  fs << "minThreshold" << param_int.minThreshold;
  fs << "maxThreshold" << param_int.maxThreshold;

  fs << "minRepeatability" << static_cast<int>(param_int.minRepeatability);
  fs << "minDistBetweenBlobs" << param_int.minDistBetweenBlobs;

  fs << "filterByColor" << static_cast<int>(param_int.filterByColor);
  fs << "blobColor" << static_cast<int>(param_int.blobColor);

  fs << "filterByArea" << static_cast<int>(param_int.filterByArea);
  fs << "minArea" << param_int.minArea;
  fs << "maxArea" << param_int.maxArea;

  fs << "filterByCircularity"
     << static_cast<int>(param_int.filterByCircularity);
  fs << "minCircularity" << param_int.minCircularity / 5.;
  fs << "maxCircularity" << param_int.maxCircularity / 1.;

  fs << "filterByInertia" << static_cast<int>(param_int.filterByInertia);
  fs << "minInertiaRatio" << param_int.minInertiaRatio / 10.;
  fs << "maxInertiaRatio" << param_int.maxInertiaRatio / 1.;

  fs << "filterByConvexity" << static_cast<int>(param_int.filterByConvexity);
  fs << "minConvexity" << param_int.minConvexity / 5.;
  fs << "maxConvexity" << param_int.maxConvexity / 1.;
  SPDLOG_WARN("Wrote params.");
}