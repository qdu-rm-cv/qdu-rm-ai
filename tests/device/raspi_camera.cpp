#include "raspi_camera.hpp"

#include <fstream>

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

namespace {
const std::string kPATH = "../../../image/test.png";
} // namespace

TEST(TestRaspiCamera, TestCapture) {
  RaspiCamera cam;
  ASSERT_TRUE(cam.Open(0) == false) << "Can not open camera 0.";
  cam.Open(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  cv::Mat frame = cam.GetFrame();
  ASSERT_FALSE(frame.empty()) << "Can not get frame from camera.";

  cv::imwrite(kPATH, frame);
  std::ifstream f(kPATH);
  ASSERT_TRUE(f.good()) << "Can not write frame to file.";
  f.close();

  cv::Mat img = imread(kPATH, cv::IMREAD_COLOR);
  ASSERT_FALSE(img.empty()) << "Can not opening image after wrote.";
}
