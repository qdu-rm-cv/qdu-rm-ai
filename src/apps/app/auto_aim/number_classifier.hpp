// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"

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
                   const std::vector<std::string>& ignore_classes = {}) {
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
      class_names_.push_back(line);
    }
  }

  void classify(std::vector<Armor>& armors, const cv::Mat frame) {
    for (auto& armor : armors) {
      cv::Mat image = armor.Face(frame);

      // Normalize
      image = image / 255.0;

      // Create blob from image
      cv::Mat blob;
      cv::dnn::blobFromImage(image, blob, 1., cv::Size(28, 20));

      // Set the input blob for the neural network
      net_.setInput(blob);
      // Forward pass the image blob through the model
      cv::Mat outputs = net_.forward();

      // Do softmax
      float max_prob =
          *std::max_element(outputs.begin<float>(), outputs.end<float>());
      cv::Mat softmax_prob;
      cv::exp(outputs - max_prob, softmax_prob);
      float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
      softmax_prob /= sum;

      double confidence;
      cv::Point class_id_point;
      minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr,
                &class_id_point);
      int label_id = class_id_point.x;

      armor.confidence_ = confidence;
      armor.number_ = class_names_[label_id];
      armor.armor_type_ = armor.GetArmorType();
      std::stringstream result_ss;
      result_ss << armor.number_ << ": " << std::fixed << std::setprecision(1)
                << armor.confidence_ * 100.0 << "%";
      armor.classfication_result_ = result_ss.str();
    }

    armors.erase(
        std::remove_if(armors.begin(), armors.end(),
                       [this](const Armor& armor) {
                         if (armor.confidence_ < threshold ||
                             armor.number_ == "Negative") {
                           return true;
                         }

                         for (const auto& ignore_class : ignore_classes_) {
                           if (armor.number_ == ignore_class) {
                             return true;
                           }
                         }

                         bool mismatch_armor_type = false;
                         if (armor.armor_type_ == LARGE) {
                           mismatch_armor_type = armor.number_ == "Outpost" ||
                                                 armor.number_ == "2" ||
                                                 armor.number_ == "Guard";
                         } else if (armor.armor_type_ == SMALL) {
                           mismatch_armor_type =
                               armor.number_ == "1" || armor.number_ == "Base";
                         }
                         return mismatch_armor_type;
                       }),
        armors.end());
  }

  double threshold;

 private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
