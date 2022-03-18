#pragma once
#include "common.hpp"
#include "opencv2/opencv.hpp"
#include "trt_detector.hpp"

class RadarDetector {
 private:
  TrtDetector detector_;

  bool Search(std::vector<cv::Point2f> contour, cv::Rect2f anchor);
  game::Alert DetectRegion(const cv::Mat& frame);

 public:
  RadarDetector();
  RadarDetector(const std::string& onnx_file_path, float conf_thresh = 0.5f,
                float nms_thresh = 0.5f);
  ~RadarDetector();

  game::Alert Detect(const cv::Mat& frame);
};
