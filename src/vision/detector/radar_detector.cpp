#include "radar_detector.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kDIFF_HEIGHT = 0.4;
const double kDIFF_TOP = 1. / 5.;
const double kDIFF_BOTTOM = 1. / 7.;
const double kDIFF_ALL = 1 - kDIFF_TOP - kDIFF_BOTTOM;

const std::vector<cv::Point2f> kAREA_ENEMY_BUFF = {
    cv::Point(100, 100), cv::Point(150, 100), cv::Point(150, 150),
    cv::Point(100, 150)};
const std::vector<cv::Point2f> kAREA_ENEMY_SNIPE = {
    cv::Point(200, 200), cv::Point(250, 200), cv::Point(250, 250),
    cv::Point(200, 250)};
const std::vector<cv::Point2f> kAREA_ENEMY_SLOPE = {
    cv::Point(300, 300), cv::Point(350, 300), cv::Point(350, 350),
    cv::Point(300, 350)};
const std::vector<cv::Point2f> kAREA_SELF_OUTPOST = {
    cv::Point(400, 400), cv::Point(450, 400), cv::Point(450, 450),
    cv::Point(400, 450)};
const std::vector<cv::Point2f> kAREA_SELF_SENTRY = {
    cv::Point(500, 500), cv::Point(550, 500), cv::Point(550, 550),
    cv::Point(500, 550)};
const std::vector<cv::Point2f> kAREA_SELF_BASE = {
    cv::Point(600, 600), cv::Point(650, 600), cv::Point(650, 650),
    cv::Point(600, 650)};

}  // namespace

bool RadarDetector::Search(std::vector<cv::Point2f> contour,
                           cv::Rect2f anchor) {
  double bottom = anchor.br().y;
  double left = anchor.tl().x;
  double height = kDIFF_HEIGHT * anchor.height;

  std::vector<cv::Point2f> points{
      cv::Point2f(left + 0.25 * anchor.width, bottom - 0.25 * height),
      cv::Point2f(left + 0.5 * anchor.width, bottom - 0.5 * height),
      cv::Point2f(left + 0.75 * anchor.width, bottom - 0.75 * height),
      cv::Point2f(
          left + 0.25 * kDIFF_ALL * anchor.width + kDIFF_TOP * anchor.width,
          bottom - 0.75 * height),
      cv::Point2f(
          left + 0.75 * kDIFF_ALL * anchor.width + kDIFF_TOP * anchor.width,
          bottom - 0.25 * height)};

  int count = 0;
  for (auto& point : points)
    if (cv::pointPolygonTest(contour, point, false) > 0) count++;

  if (count >= 3)
    return true;
  else
    return false;
}

Alert RadarDetector::DetectRegion(const cv::Mat& frame) {
  Alert alert;
  auto detections = detector_.Infer(frame);

  if (detections.empty()) {
    SPDLOG_ERROR("Detections is empty.");
    return;
  }
  for (auto& detection : detections) {
    cv::Rect2f anchor(detection.x_ctr - detection.w / 2.,
                      detection.y_ctr - detection.h / 2., detection.w,
                      detection.h);

    if (alert.enemy_buff != true) {
      alert.enemy_buff = Search(kAREA_ENEMY_BUFF, anchor);
    }
    if (alert.enemy_snipe != true) {
      alert.enemy_snipe = Search(kAREA_ENEMY_SNIPE, anchor);
    }
    if (alert.enemy_slope != true) {
      alert.enemy_slope = Search(kAREA_ENEMY_SLOPE, anchor);
    }
    if (alert.self_base != true) {
      alert.self_base = Search(kAREA_SELF_BASE, anchor);
    }
    if (alert.self_outpost != true) {
      alert.self_outpost = Search(kAREA_SELF_OUTPOST, anchor);
    }
    if (alert.self_sentry != true) {
      alert.self_sentry = Search(kAREA_SELF_SENTRY, anchor);
    }
  }
  SPDLOG_DEBUG("All Regions have been Detected.");
  return alert;
}

RadarDetector::RadarDetector() { SPDLOG_TRACE("Constructed."); }

RadarDetector::RadarDetector(const std::string& onnx_file_path,
                             float conf_thresh, float nms_thresh) {
  detector_.SetOnnxPath(onnx_file_path);
  detector_.Init(conf_thresh, nms_thresh);
  SPDLOG_TRACE("Constructed.");
}

RadarDetector::~RadarDetector() { SPDLOG_TRACE("Destructed."); }

Alert RadarDetector::Detect(const cv::Mat& frame) {
  SPDLOG_DEBUG("Detecting");
  Alert result = DetectRegion(frame);
  SPDLOG_DEBUG("Detected.");
  return result;
}
