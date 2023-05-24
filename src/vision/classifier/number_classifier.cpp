// Copyright 2022 Chen Jun
// Licensed under the MIT License.
// Thanks to Mr.Chen
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor.hpp"
#include "number_classifier.hpp"

NumberClassifier::NumberClassifier(
    const std::string& model_path, const std::string& label_path,
    const double thre, const std::vector<std::string>& ignore_classes)
    : threshold(thre), ignore_classes_(ignore_classes) {
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}
void NumberClassifier::classify(std::vector<Armor>& armors,
                                const cv::Mat frame) {
  for (auto& armor : armors) {
    cv::Mat image = armor.number_img_;  // armor.Face(frame);

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
