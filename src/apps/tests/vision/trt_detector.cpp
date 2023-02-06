#include "trt_detector.hpp"

#include "gtest/gtest.h"
#include "log.hpp"
#include "opencv2/opencv.hpp"

TEST(TestNN, ExampleTest) { EXPECT_EQ(1, 1); }

TEST(TestNN, TestTRT) {
  component::logger::SetLogger();
  TrtDetector detector(std::string(kPATH_RUNTIME + "best.onnx"));
  detector.TestInfer();
}
