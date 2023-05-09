#pragma once

#include "armor_detector.hpp"
#include "armor_param.hpp"
#include "ui.hpp"

class ArmorUIParam : private UI {
 private:
  ArmorDetector detector_;
  ArmorParam armor_param_;

 public:
  ArmorUIParam(const std::string& log_path, const std::string& param_path,
               const std::string& window = "ui_setting")
      : UI(log_path, param_path, window) {
    SPDLOG_WARN("***** Setting Up ArmorUIParam System. *****");

    /* 初始化设备 */
    detector_.SetEnemyTeam(game::Team::kBLUE);
    armor_param_.Read(param_path_);
  }

  ~ArmorUIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down ArmorUIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(window_handle_, 1);

    cv::createTrackbar("binary_th", window_handle_,
                       &armor_param_.parami_.binary_th, 255, 0);
    cv::createTrackbar("contour_size_low_th", window_handle_,
                       &armor_param_.parami_.contour_size_low_th, 50);
    cv::createTrackbar("contour_area_low_th", window_handle_,
                       &armor_param_.parami_.contour_area_low_th, 2000);
    cv::createTrackbar("contour_area_high_th", window_handle_,
                       &armor_param_.parami_.contour_area_high_th, 500);
    cv::createTrackbar("bar_area_low_th", window_handle_,
                       &armor_param_.parami_.bar_area_low_th, 2000);
    cv::createTrackbar("bar_area_high_th", window_handle_,
                       &armor_param_.parami_.bar_area_high_th, 500);
    cv::createTrackbar("angle_high_th", window_handle_,
                       &armor_param_.parami_.angle_high_th, 600);
    cv::createTrackbar("aspect_ratio_low_th", window_handle_,
                       &armor_param_.parami_.aspect_ratio_low_th, 100);
    cv::createTrackbar("aspect_ratio_high_th", window_handle_,
                       &armor_param_.parami_.aspect_ratio_high_th, 1000);

    cv::createTrackbar("angle_diff_th", window_handle_,
                       &armor_param_.parami_.angle_diff_th, 30000);
    cv::createTrackbar("length_diff_th", window_handle_,
                       &armor_param_.parami_.length_diff_th, 1000);
    cv::createTrackbar("height_diff_th", window_handle_,
                       &armor_param_.parami_.height_diff_th, 200);
    cv::createTrackbar("center_y_dist", window_handle_,
                       &armor_param_.parami_.center_y_dist, 1000);
    cv::createTrackbar("area_diff_th", window_handle_,
                       &armor_param_.parami_.area_diff_th, 5000);
    cv::createTrackbar("center_dist_low_th", window_handle_,
                       &armor_param_.parami_.center_dist_low_th, 5000);
    cv::createTrackbar("center_dist_high_th", window_handle_,
                       &armor_param_.parami_.center_dist_high_th, 9000);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);
    cv::Mat frame;

    while (true) {
      cam_.GetFrame(frame);
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.params_ = armor_param_.TransformToDouble();
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 3);

      cv::imshow(window_handle_, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's' || key == 'S') {
        armor_param_.Write(param_path_);
      } else if (key == 'q' || key == 27 || key == 'Q') {
        cv::destroyAllWindows();
        return;
      }
    }
  }
};
