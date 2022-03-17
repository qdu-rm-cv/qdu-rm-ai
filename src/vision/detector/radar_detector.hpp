#pragma once

#include "opencv2/opencv.hpp"
#include "trt_detector.hpp"

struct Alert {
  bool enemy_buff;
  bool enemy_snipe;
  bool enemy_slope;
  bool self_outpost;
  bool self_sentry;
  bool self_base;

  Alert() {
    enemy_buff = false;
    enemy_snipe = false;
    enemy_slope = false;
    self_outpost = false;
    self_sentry = false;
    self_base = false;
  }
};

class RadarDetector {
 private:
  TrtDetector detector_;

  bool Search(std::vector<cv::Point2f> contour, cv::Rect2f anchor);
  Alert DetectRegion();

 public:
  RadarDetector();
  RadarDetector(const std::string& onnx_file_path, float conf_thresh = 0.5f,
                float nms_thresh = 0.5f);
  ~RadarDetector();

  Alert Detect(const cv::Mat& frame);
};
