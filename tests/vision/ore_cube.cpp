#include "ore_cube.hpp"

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestOreCube) {
  cv::RotatedRect rect(cv::Point(50, 50), cv::Size(40, 40), 0);
  OreCube cube(rect);
  ASSERT_FLOAT_EQ(cube.GetRadius(), 20 * sqrt(3));
}
