#include "buff_detector.hpp"

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestBuffDetector) {
  BuffDetector buff_detector(kPATH_RUNTIME + "RMUT2021_Buff.json",
                             game::Team::kBLUE);

  if (!algo::FileExist(kPATH_IMAGE + "test_buff.png")) return;
  cv::Mat frame = cv::imread(kPATH_IMAGE + "test_buff.png", cv::IMREAD_COLOR);
  ASSERT_FALSE(frame.empty()) << "Can not opening image.";

  buff_detector.Detect(frame);
  buff_detector.VisualizeResult(frame, 2);

  cv::Mat result = frame.clone();
  cv::imwrite(kPATH_IMAGE + "test_buff_result.png", result);
  SUCCEED();
}

TEST(TestVision, TestBuffDetectorVideo) {
  BuffDetector buff_detector(kPATH_RUNTIME + "RMUT2021_Buff.json",
                             game::Team::kBLUE);

  if (!algo::FileExist("../../../../../../redbuff01.avi")) return;
  cv::VideoCapture cap("../../../../../../redbuff01.avi");
  cv::Mat frame;
  ASSERT_TRUE(cap.isOpened()) << "cap not opened";
  while (cap.isOpened()) {
    cap >> frame;
    ASSERT_FALSE(frame.empty()) << "Can not opening image.";
    buff_detector.Detect(frame);
    buff_detector.VisualizeResult(frame, 5);
#if WITH_UI
    cv::imshow("result", frame);
    cv::waitKey(1);
#else
    break;
#endif
  }
  cap.release();
}
