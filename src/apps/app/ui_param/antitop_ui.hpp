#pragma once

#include "anti_whippingtop.hpp"
#include "antitop_param.hpp"
#include "ui.hpp"
#include "armor_detector.hpp"
#include "armor_param.hpp"
#include "hik_camera.hpp"
#include "raspi_camera.hpp"

class AntiTopUIParam : private UI {
 private:
  ArmorDetector detector_;
  AntiWhippingTop antitop_;
  AntiTopParam antitop_param_;
  ArmorParam armor_param_;
  std::string param_path_antitop_, param_path_armor_, window_handle_;

 public:
  AntiTopUIParam(const std::string& log_path,
                 const std::string& param_path_armor,
                 const std::string& param_path_antitop,
                 const std::string& window = "ui_setting")
      : UI(log_path, param_path_armor, window),
        param_path_antitop_(param_path_antitop),
        param_path_armor_(param_path_armor),
        window_handle_(window) {
    SPDLOG_WARN("***** Setting Up AntiTopUIParam System. *****");

    /* 初始化设备 */
    // cam_.Open(0);
    // cam_.Setup(640, 480);
    detector_.SetEnemyTeam(game::Team::kBLUE);
    armor_param_.Read(param_path_armor);
    antitop_.SetEnemyTeam(game::Team::kBLUE);
    antitop_param_.Read(param_path_antitop);
  }

  ~AntiTopUIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down AntiTopUIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(window_handle_, 1);

    cv::createTrackbar("parami_.detector_param_th", window_handle_,
                       &antitop_param_.parami_.detector_param_th, 30);
    cv::createTrackbar("missing_frame_param_th", window_handle_,
                       &antitop_param_.parami_.missing_frame_param_th, 50);
    cv::createTrackbar("whipping_top_param_th", window_handle_,
                       &antitop_param_.parami_.whipping_top_param_th, 30);
    cv::createTrackbar("center_height_diff_low", window_handle_,
                       &antitop_param_.parami_.center_height_diff_low, 300);
    cv::createTrackbar("center_height_diff_high", window_handle_,
                       &antitop_param_.parami_.center_height_diff_high, 1000);
    cv::createTrackbar("center_width_diff_low", window_handle_,
                       &antitop_param_.parami_.center_width_diff_low, 300);
    cv::createTrackbar("center_width_diff_high", window_handle_,
                       &antitop_param_.parami_.center_width_diff_high, 1000);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);
    cv::Mat frame;
    // cv::VideoCapture capture("../../../../runtime/Unknown_Moving_Blue1.mp4");
    while (true) {
      cam_.GetFrame(frame);
      // capture >> frame;
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.params_ = armor_param_.TransformToDouble();
      antitop_.params_ = antitop_param_.TransformToDouble();
      auto armors_ = detector_.Detect(frame);
      antitop_.SetDetects(armors_);
      // antitop_.IsWhipping(armors_);
      detector_.VisualizeResult(frame, 3);
      antitop_.Predict();
      antitop_.VisualizePrediction(frame, 3);
      // antitop_.VisualizePrediction(frame, 3);
      SPDLOG_INFO("detector_param_ is {}", antitop_.detector_param_);
      SPDLOG_INFO("missing_frame_param_ is {}", antitop_.missing_frame_param_);
      SPDLOG_INFO("whipping_top_param_ is {}", antitop_.whipping_top_param_);
      cv::imshow(window_handle_, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's' || key == 'S') {
        antitop_param_.Write(param_path_antitop_);
      } else if (key == 'q' || key == 27 || key == 'Q') {
        cv::destroyAllWindows();
        return;
      }
    }
  }
};
