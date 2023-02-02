#include "armor_classifier.hpp"

#include "armor.hpp"
#include "common.hpp"
#include "gtest/gtest.h"

namespace {

ArmorClassifier armor_classifier("../../../runtime/armor_classifier.onnx",
                                 "../../../runtime/armor_classifier_lable.json",
                                 cv::Size(28, 28));

game::Model Classify(const std::string& path) {
  cv::Mat f = cv::imread(path);
  Armor armor(cv::RotatedRect(cv::Point2f(0, 0), cv::Point2f(f.cols, 0),
                              cv::Point2f(f.cols, f.rows)));
  armor_classifier.ClassifyModel(armor, f);
  return armor.GetModel();
}

}  // namespace

TEST(TestVision, TestArmorClassifier) {
  if (algo::FileExist("../../../assets/image/test_classifier_0.png"))
    ASSERT_TRUE(Classify("../../../assets/image/test_classifier_0.png") ==
                game::Model::kINFANTRY);
  else if (algo::FileExist("../../../assets/image/test_classifier_1.png"))
    ASSERT_TRUE(Classify("../../../assets/image/test_classifier_1.png") ==
                game::Model::kENGINEER);
  else if (algo::FileExist("../../../assets/image/test_classifier_2.png"))
    ASSERT_TRUE(Classify("../../../assets/image/test_classifier_2.png") ==
                game::Model::kHERO);
  else if (algo::FileExist("../../../assets/image/test_classifier_3.png"))
    ASSERT_TRUE(Classify("../../../assets/image/test_classifier_3.png") ==
                game::Model::kHERO);
  else
    return;
}

TEST(TestVision, TestArmorClassifierInput) {
  if (!algo::FileExist("../../../assets/image/p2.png")) return;

  cv::Mat f = cv::imread("../../../assets/image/p2.png");
  Armor armor(cv::RotatedRect(cv::Point2f(0, 0), cv::Point2f(f.cols, 0),
                              cv::Point2f(f.cols, f.rows)));

  cv::imwrite("../../../assets/image/test_face.png", armor.Face(f));
  cv::Mat nn_input;
  cv::resize(armor.Face(f), nn_input, cv::Size(28, 28));
  cv::imwrite("../../../assets/image/test_nn_input.png", nn_input);
}
