#define async
#ifdef async

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "app.hpp"
#include "armor.hpp"
#include "armor_classifier.hpp"
#include "async_armor_detector.hpp"
#include "behavior.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"
// int counter = 0;
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
    for (auto class_names : class_names_) {
      SPDLOG_ERROR("class_names:{}", class_names);
    }
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
      // cv::Point2f lights_vertices[4] = {
      //     armor.left_light.bottom, armor.left_light.top,
      //     armor.right_light.top, armor.right_light.bottom};
      cv::Point2f lights_vertices[4];
      std::vector<cv::Point2f> image_vertices = armor.ImageVertices();
      for (int i = 0; i < 4; i++) {
        lights_vertices[i] = image_vertices[i];
      }
      cv::Point2f left_light_bar_center =
          (image_vertices[0] + image_vertices[1]) / 2;
      cv::Point2f right_light_bar_center =
          (image_vertices[3] + image_vertices[2]) / 2;
      // cv::Point2f left_bar_top =
      //     left_light_bar_center - (image_vertices[0] - image_vertices[1]) /
      //     4;
      // cv::Point2f left_bar_bottom =
      //     left_light_bar_center + (image_vertices[0] - image_vertices[1]) /
      //     4;
      // cv::Point2f right_bar_top =
      //     right_light_bar_center - (image_vertices[2] - image_vertices[3]) /
      //     4;
      // cv::Point2f right_bar_bottom =
      //     right_light_bar_center + (image_vertices[2] - image_vertices[3]) /
      //     4;
      // lights_vertices[0] =
      //     left_light_bar_center +
      //     (image_vertices[0] - image_vertices[1]) / 4;  // left_bar_bottom;
      // lights_vertices[1] =
      //     left_light_bar_center -
      //     (image_vertices[0] - image_vertices[1]) / 4;  // left_bar_top;
      // lights_vertices[2] =
      //     right_light_bar_center -
      //     (image_vertices[2] - image_vertices[3]) / 4;  // right_bar_top;
      // lights_vertices[3] =
      //     right_light_bar_center +
      //     (image_vertices[2] - image_vertices[3]) / 4;  // right_bar_bottom;
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
      // cv::Mat a = src, clone();
      // for (int i = 1; i <= 4; i++) {
      //   cv::line(a, lights_vertices[(i - 1) % 4], lights_vertices[i % 4],
      //            cv::Scalar(100, 100, 100));
      // }

      cv::Mat number_image;
      auto rotation_matrix =
          cv::getPerspectiveTransform(lights_vertices, target_vertices);
      cv::warpPerspective(src, number_image, rotation_matrix,
                          cv::Size(warp_width, warp_height));

      //进行腐蚀操作
      // cv::Mat element =
      //     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
      // erode(src, number_image, element);

      // Get ROI
      // cv::imshow("--------", number_image);

      number_image = number_image(
          cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

      // cv::imshow("----", number_image);
      /*  cv::imread("/home/yang/Desktop/new/qdu-rm-ai/runtime/000455.png");*/
      // Binarize
      cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
      cv::threshold(number_image, number_image, 0, 255,
                    cv::THRESH_BINARY | cv::THRESH_OTSU);
      armor.number_img_ = number_image;
      cv::imshow("-----------", number_image);
      // std::string path;
      // path = cv::format(
      //     "/home/yang/Documents/rm_classifier_training-0.1/datasets/3/%d.png",
      //     counter);
      // switch (cv::waitKey(10)) {
      //   case 's':
      //     cv::imwrite(path, armor.number_img_);
      //     counter++;
      //     if (counter > 500) system("pause");
      //     break;
      //   case 'q':
      //     break;
      // }
    }
  }

  void classify(std::vector<Armor>& armors, const cv::Mat frame) {
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
                         SPDLOG_ERROR("------------" + armor.number_);
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

  double threshold;

 private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
class AutoAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  ArmorDetectorAsync detector_async_;
  Compensator compensator_;
  Behavior manager_;

  component::Recorder recorder_ = component::Recorder("AutoAimThread");
  ArmorClassifier classifier_;
  // kkk
 public:
  explicit AutoAim(const std::string& log_path)
      : App(log_path), detector_async_(ArmorDetectorAsync(2)) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    // robot_.Init("/dev/ttyUSB0");
    cam_.Open(0);
    cam_.Setup(kIMAGE_WIDTH, kIMAGE_HEIGHT);
    detector_async_.LoadParams(kPATH_RUNTIME + "RMUL2022_Armor.json");
    compensator_.LoadCameraMat(kPATH_RUNTIME + "MV-CA016-10UC-6mm_1.json");
    // classifier_.LoadModel(kPATH_RUNTIME + "mpl.onnx");
    // classifier_.LoadLable(kPATH_RUNTIME + "label.json");
    // classifier_.SetInputSize(cv::Size(28, 28));

    // do {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    // detector_async_.SetEnemyTeam(robot_.GetEnemyTeam());
    detector_async_.SetEnemyTeam(game::Team::kBLUE);
  }

  ~AutoAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");
    cv::Mat frame;
    tbb::concurrent_vector<Armor> armors;
    detector_async_.Start();
    std::string path1 = kPATH_RUNTIME + "mlp.onnx";
    std::string path2 = kPATH_RUNTIME + "label.txt";
    std::vector<std::string> mask = {"Negative"};
    NumberClassifier num_classfier(
        "/home/yang/Desktop/new/qdu-rm-ai/runtime/mlp.onnx", path2, 0.8);
    while (1) {
      if (!cam_.GetFrame(frame)) continue;

      detector_async_.PutFrame(frame);
      if (!detector_async_.GetResult(armors)) continue;

      // for (auto armor : armors) {
      //   classifier_.ClassifyModel(armor, frame);
      // }
      std::vector<Armor> armors_t;
      for (auto armor : armors) {
        armors_t.push_back(armor);
      }
      // cv::imshow("face", armors_t.front().number_img_);
      SPDLOG_ERROR("11111111111111111111111");
      SPDLOG_ERROR("armor.num.1 {}", armors_t.size());
      num_classfier.extractNumbers(frame, armors_t);
      num_classfier.classify(armors_t, frame);

      tbb::concurrent_vector<Armor> armors2;
      for (auto armor : armors_t) {
        armors2.push_back(armor);
      }
      SPDLOG_ERROR("armor.num {}", armors2.size());
      if (!armors2.empty())
        SPDLOG_ERROR("classfier {}");  // pai xv xou hua
      else {
        SPDLOG_ERROR("EMPTY!!!");
        continue;
      }
      compensator_.Apply(armors2, /*robot_.GetBalletSpeed()*/ 999,
                         component::Euler(0, 0, 0) /* robot_.GetEuler()*/,
                         game::AimMethod::kARMOR);
      // manager_.Aim(armors.front().GetAimEuler());
      // robot_.Pack(manager_.GetData(), 9999);

      SPDLOG_WARN("pack");
      // recorder_.Record();
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  AutoAim auto_aim("logs/auto_aim.log");
  auto_aim.Run();

  return EXIT_SUCCESS;
}

#else

#include "app.hpp"
#include "armor_detector.hpp"
#include "behavior.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class AutoAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  ArmorDetector detector_;
  Compensator compensator_;
  Behavior manager_;

  component::Recorder recorder_ = component::Recorder("AutoAimThread");

 public:
  explicit AutoAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init();
    cam_.Open(0);
    cam_.Setup(kIMAGE_WIDTH, kIMAGE_HEIGHT);
    detector_.LoadParams(kPATH_RUNTIME + "RMUL2022_Armor.json");
    compensator_.LoadCameraMat(kPATH_RUNTIME + "MV-CA016-10UC-6mm_1.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    detector_.SetEnemyTeam(robot_.GetEnemyTeam());
    // detector_.SetEnemyTeam(game::Team::kBLUE);
  }

  ~AutoAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");
    cv::Mat frame;

    while (1) {
      cam_.GetFrame(frame);
      if (frame.empty()) continue;
      auto armors = detector_.Detect(frame);

      if (armors.size() != 0) {
        compensator_.Apply(armors, robot_.GetBalletSpeed(), robot_.GetEuler());
        manager_.Aim(armors.front().GetAimEuler());
        robot_.Pack(manager_.GetData(), 9999);

        detector_.VisualizeResult(frame, 10);
      }
      cv::imshow("show", frame);
      if (' ' == cv::waitKey(10)) {
        cv::waitKey(0);
      }
      recorder_.Record();
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  AutoAim auto_aim("logs/auto_aim.log");
  auto_aim.Run();

  return EXIT_SUCCESS;
}

#endif
