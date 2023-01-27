#include "guidinglight_detector.hpp"

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestGuidngLightDetector) {
  GuidingLightDetector light_detector("../../../runtime/light_params.json");

  cv::Mat img = imread("../../../image/test_light.png", cv::IMREAD_COLOR);
  ASSERT_FALSE(img.empty()) << "Can not opening image.";

  tbb::concurrent_vector<GuidingLight> light = light_detector.Detect(img);

  cv::Mat result = img.clone();
  light_detector.VisualizeResult(result, 2);
  cv::imwrite("../../../image/test_origin_light.png", result);

  cv::resize(img, img, cv::Size(640, 426));

  light = light_detector.Detect(img);
  result = img.clone();
  light_detector.VisualizeResult(result, 1);
  cv::imwrite("../../../image/test_resized_light.png", result);
}
