#include "orecube_detector.hpp"

#include "common.hpp"
#include "gtest/gtest.h"
#include "log.hpp"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestOreCubeDetector) {
  component::logger::SetLogger();
  OreCubeDetector detector(kPATH_RUNTIME + "RMUT2022_OreCube.json");

  if (!algo::FileExist("../../../../../../cube01.avi")) return;
  cv::VideoCapture cap("../../../../../../cube01.avi");
  ASSERT_TRUE(cap.isOpened()) << "cap not opened";

  cv::Mat frame;

  while (cap.isOpened()) {
    cap >> frame;
    ASSERT_FALSE(frame.empty()) << "Can not opening image.";
    if (frame.empty()) SPDLOG_ERROR("Empty");
    detector.Detect(frame);
    detector.VisualizeResult(frame, 3);
#if WITH_UI
    cv::imshow("show", frame);
    auto key = cv::waitKey(10);
    if (key == 'q') {
      cv::waitKey(0);
      cap.release();
      return;
    } else if (key == ' ') {
      cv::waitKey(0);
    }
#else
    break;
#endif
  }
}
