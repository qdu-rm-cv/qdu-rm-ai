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
  for (auto class_names : class_names_) {
    SPDLOG_ERROR("class_names:{}", class_names);
  }
  ignore_classes_ = ignore_classes;
}
void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors) {
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto& armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4];
    std::vector<cv::Point2f> image_vertices = armor.ImageVertices();
    for (int i = 0; i < 4; i++) {
      lights_vertices[i] = image_vertices[i];
    }
    cv::Point2f left_light_bar_center =
        (image_vertices[0] + image_vertices[1]) / 2;
    cv::Point2f right_light_bar_center =
        (image_vertices[3] + image_vertices[2]) / 2;

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width =
        armor.armor_type_ == SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };

    cv::Mat number_image;
    auto rotation_matrix =
        cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix,
                        cv::Size(warp_width, warp_height));

    number_image = number_image(
        cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);
    armor.number_img_ = number_image;
    cv::imshow("target number", number_image);
  }
}
void NumberClassifier::classify(std::vector<Armor>& armors,
                                const cv::Mat frame) {
  for (auto& armor : armors) {
    cv::Mat image = armor.number_img_.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

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
    int label_id = class_id_point.x;  // id
    armor.confidence_ = confidence;
    armor.number_ = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << armor.number_ << ": " << std::fixed << std::setprecision(1)
              << armor.confidence_ * 100.0 << "%";
    armor.classfication_result_ = result_ss.str();
  }

  armors.erase(
      std::remove_if(armors.begin(), armors.end(),
                     [this](const Armor& armor) {
                       if (armor.confidence_ < threshold) {
                         return true;
                       }

                       for (const auto& ignore_class : ignore_classes_) {
                         if (armor.number_ == ignore_class) {
                           return true;
                         }
                       }

                       bool mismatch_armor_type = false;
                       if (armor.armor_type_ == ArmorType::LARGE) {
                         mismatch_armor_type = armor.number_ == "outpost" ||
                                               armor.number_ == "2" ||
                                               armor.number_ == "guard";
                       } else if (armor.armor_type_ == ArmorType::SMALL) {
                         mismatch_armor_type =
                             armor.number_ == "1" || armor.number_ == "base";
                       }
                       return mismatch_armor_type;
                     }),
      armors.end());
}
