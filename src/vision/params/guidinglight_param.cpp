#include "guidinglight_param.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

bool GuidingLightParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    parami_.thresholdStep = fs["thresholdStep"];
    parami_.minThreshold = fs["minThreshold"];
    parami_.maxThreshold = fs["maxThreshold"];

    parami_.minRepeatability = int(fs["minRepeatability"]);
    parami_.minDistBetweenBlobs = fs["minDistBetweenBlobs"];

    parami_.filterByColor = int(fs["filterByColor"]) != 0 ? true : false;
    parami_.blobColor = int(fs["blobColor"]);
    parami_.filterByArea = int(fs["filterByArea"]) != 0 ? true : false;
    parami_.minArea = fs["minArea"];
    parami_.maxArea = fs["maxArea"];

    parami_.filterByCircularity =
        int(fs["filterByCircularity"]) != 0 ? true : false;
    parami_.minCircularity = fs["minCircularity"];
    parami_.maxCircularity = fs["maxCircularity"];

    parami_.filterByInertia = int(fs["filterByInertia"]) != 0 ? true : false;
    parami_.minInertiaRatio = fs["minInertiaRatio"];
    parami_.maxInertiaRatio = fs["maxInertiaRatio"];

    parami_.filterByConvexity =
        int(fs["filterByConvexity"]) != 0 ? true : false;
    parami_.minConvexity = fs["minConvexity"];
    parami_.maxConvexity = fs["maxConvexity"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

cv::SimpleBlobDetector::Params GuidingLightParam::TransformToDouble() {
  paramd_.thresholdStep = parami_.thresholdStep / 1.;
  paramd_.minThreshold = parami_.minThreshold / 1.;
  paramd_.maxThreshold = parami_.maxThreshold / 1.;

  paramd_.minRepeatability = parami_.minRepeatability;
  paramd_.minDistBetweenBlobs = parami_.minDistBetweenBlobs / 1.;

  paramd_.filterByColor = parami_.filterByColor;
  paramd_.blobColor = parami_.blobColor;

  paramd_.filterByArea = parami_.filterByArea;
  paramd_.minArea = parami_.minArea / 1.;
  paramd_.maxArea = parami_.maxArea / 1.;

  paramd_.filterByCircularity = parami_.filterByCircularity;
  paramd_.minCircularity = parami_.minCircularity / 20.;
  paramd_.maxCircularity = parami_.maxCircularity / 20.;

  paramd_.filterByInertia = parami_.filterByInertia;
  paramd_.minInertiaRatio = parami_.minInertiaRatio / 20.;
  paramd_.maxInertiaRatio = parami_.maxInertiaRatio / 20.;

  paramd_.filterByConvexity = parami_.filterByConvexity;
  paramd_.minConvexity = parami_.minConvexity / 20.;
  paramd_.maxConvexity = parami_.maxConvexity / 20.;
  return paramd_;
}

void GuidingLightParam::Write(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "thresholdStep" << paramd_.thresholdStep;
  fs << "minThreshold" << paramd_.minThreshold;
  fs << "maxThreshold" << paramd_.maxThreshold;

  fs << "minRepeatability" << static_cast<int>(paramd_.minRepeatability);
  fs << "minDistBetweenBlobs" << paramd_.minDistBetweenBlobs;

  fs << "filterByColor" << static_cast<int>(paramd_.filterByColor);
  fs << "blobColor" << static_cast<int>(paramd_.blobColor);

  fs << "filterByArea" << static_cast<int>(paramd_.filterByArea);
  fs << "minArea" << paramd_.minArea;
  fs << "maxArea" << paramd_.maxArea;

  fs << "filterByCircularity" << static_cast<int>(paramd_.filterByCircularity);
  fs << "minCircularity" << paramd_.minCircularity;
  fs << "maxCircularity" << paramd_.maxCircularity;

  fs << "filterByInertia" << static_cast<int>(paramd_.filterByInertia);
  fs << "minInertiaRatio" << paramd_.minInertiaRatio;
  fs << "maxInertiaRatio" << paramd_.maxInertiaRatio;

  fs << "filterByConvexity" << static_cast<int>(paramd_.filterByConvexity);
  fs << "minConvexity" << paramd_.minConvexity;
  fs << "maxConvexity" << paramd_.maxConvexity;
  SPDLOG_WARN("Wrote params.");
}