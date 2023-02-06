#include "armor_detector.hpp"

#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

TEST(TestVision, TestArmorDetector) {
  ArmorDetector armor_detector(kPATH_RUNTIME + "test_params.json",
                               game::Team::kBLUE);

  if (!algo::FileExist(kPATH_IMAGE + "test.jpg")) return;
  cv::Mat img = imread(kPATH_IMAGE + "test.jpg", cv::IMREAD_COLOR);
  cv::resize(img, img, cv::Size(640, 480));
  ASSERT_FALSE(img.empty()) << "Can not opening image.";

  tbb::concurrent_vector<Armor> armors = armor_detector.Detect(img);
  // EXPECT_EQ(armors.size(), 6) << "Can not detect armor in original image.";

  cv::Mat result = img.clone();
  armor_detector.VisualizeResult(result, 2);
  cv::imwrite(kPATH_IMAGE + "test_origin.png", result);

  auto fmt = kPATH_IMAGE + "p%ld.png";
  for (size_t i = 0; i < armors.size(); ++i) {
    cv::imwrite(cv::format(fmt.c_str(), i), armors[i].Face(img));
  }

  cv::resize(img, img, cv::Size(640, 426));

  armors = armor_detector.Detect(img);
  // EXPECT_EQ(armors.size(), 6) << "Can not detect armor in small image.";

  result = img.clone();
  armor_detector.VisualizeResult(result, 1);
  cv::imwrite(kPATH_IMAGE + "test_resized.png", result);

  armor_detector.SetEnemyTeam(game::Team::kRED);
  armors = armor_detector.Detect(img);
  // EXPECT_EQ(armors.size(), 0) << "Can not tell the enemy from ourselves.";
}
