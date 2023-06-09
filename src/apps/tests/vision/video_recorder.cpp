#include "video_recorder.hpp"

#include "gtest/gtest.h"
#include "log.hpp"
#include "object.hpp"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestVideoRecorder) {
  VideoRecorder r;
  cv::Mat black(48, 64, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat frame;
  for (int i = 0; i < 10; i++) {
    frame = black.clone();
    draw::VisualizeLabel(frame, std::to_string(i));
    SPDLOG_WARN("Draw {} now", i);
    cv::imshow("draw", frame);
    cv::waitKey(10);
    for (int j = 0; j < 60; j++) {
      r.Record(frame);
    }
  }
}
