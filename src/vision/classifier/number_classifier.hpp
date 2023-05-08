// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"

namespace rm_auto_aim {
class NumberClassifier {
 public:
  /**
   * @param model_path：模型路径
   * @param label_path：标签路径
   * @param thre:置信度阈值
   * @param ignore_classes:要筛选掉的类型
   **/
  NumberClassifier(const std::string& model_path, const std::string& label_path,
                   const double threshold,
                   const std::vector<std::string>& ignore_classes = {});

  void classify(std::vector<Armor>& armors, const cv::Mat frame);

  double threshold;

 private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
