#include "gtest/gtest.h"
#include "log.hpp"
#include "opencv2/opencv.hpp"
#include "trt_detector.hpp"

TEST(TestNN, ExampleTest) { EXPECT_EQ(1, 1); }

TEST(TestNN, TestTRT) {
  component::Logger::SetLogger();
  TrtDetector detector(std::string("../../../runtime/best.onnx"));
  detector.TestInfer();
}