#include "orecube_detector.hpp"

#include "common.hpp"
#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestOreCubeDetector) {
  RMlogger::SetLogger(spdlog::level::trace);
  OreCubeDetector detector("../../../runtime/RMUT2022_OreCube.json");
  cv::VideoCapture cap("../../../../cube01.avi");
  cv::Mat frame;
  ASSERT_TRUE(cap.isOpened()) << "cap not opened";
  while (cap.isOpened()) {
    cap >> frame;
    ASSERT_FALSE(frame.empty()) << "Can not opening image.";
    if (frame.empty()) SPDLOG_ERROR("Empty");
    detector.Detect(frame);
    detector.VisualizeResult(frame, 3);
    cv::imshow("show", frame);
    cv::waitKey(30);
  }
}