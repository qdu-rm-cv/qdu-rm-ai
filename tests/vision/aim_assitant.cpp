#include "aim_assitant.hpp"

#include "gtest/gtest.h"

TEST(TestVision, TestAimAssitant) {
  cv::Mat img = imread("../../../image/test.jpg", cv::IMREAD_COLOR);

  AimAssitant aim_assitant;
  aim_assitant.LoadParams("../../../runtime/test_params.json",
                          "../../../runtime/RMUT2021_Buff.json",
                          "../../../runtime/RMUT2022_Snipe.json");
  aim_assitant.SetRFID(game::RFID::kUNKNOWN);
  aim_assitant.SetArm(game::Arm::kHERO);

  auto armors = aim_assitant.Detect(img);
  cv::Mat result = img.clone();
  aim_assitant.VisualizeResult(result, 5);
  cv::imwrite("../../../image/aim_assitant.jpg", result);
}
