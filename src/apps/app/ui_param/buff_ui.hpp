#pragma once

#include "buff_detector.hpp"
#include "buff_param.hpp"
#include "ui.hpp"

class BuffUIParam : private UI {
 private:
  BuffDetector detector_;
  BuffParam buff_param_;

 public:
  BuffUIParam(const std::string& log_path, const std::string& param_path,
              const std::string& window = "ui_setting")
      : UI(log_path, param_path, window) {
    SPDLOG_WARN("***** Setting Up BuffUIParam System. *****");

    buff_param_.Read(param_path_);
    detector_.SetTeam(game::Team::kRED);
  }

  ~BuffUIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down BuffUIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(window_handle_, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("binary_th", window_handle_,
                       &buff_param_.parami_.binary_th, 255, 0);
    cv::createTrackbar("contour_size_low_th", window_handle_,
                       &buff_param_.parami_.contour_size_low_th, 50);
    cv::createTrackbar("rect_area_low_th", window_handle_,
                       &buff_param_.parami_.rect_area_low_th, 3000);
    cv::createTrackbar("rect_area_high_th", window_handle_,
                       &buff_param_.parami_.rect_area_high_th, 6000);
    cv::createTrackbar("contour_center_area_low_th", window_handle_,
                       &buff_param_.parami_.contour_center_area_low_th, 3000);
    cv::createTrackbar("contour_center_area_high_th", window_handle_,
                       &buff_param_.parami_.contour_center_area_high_th, 6000);
    cv::createTrackbar("rect_center_ratio_low_th / 1000", window_handle_,
                       &buff_param_.parami_.rect_center_ratio_low_th, 3000);
    cv::createTrackbar("rect_center_ratio_high_th / 1000", window_handle_,
                       &buff_param_.parami_.rect_center_ratio_high_th, 3000);
    cv::createTrackbar("rect_ratio_low_th / 1000", window_handle_,
                       &buff_param_.parami_.rect_ratio_low_th, 3000);
    cv::createTrackbar("rect_ratio_high_th / 1000", window_handle_,
                       &buff_param_.parami_.rect_ratio_high_th, 6000);
    cv::createTrackbar("armor_area_diff_th / 1000", window_handle_,
                       &buff_param_.parami_.armor_area_diff_th, 3000);
    cv::createTrackbar("armor_rect_center_dis_high_th ", window_handle_,
                       &buff_param_.parami_.armor_rect_center_dis_high_th,
                       1000);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);
    cv::Mat overimg;
    cv::Mat frame;

    while (true) {
      cam_.GetFrame(frame);
      if (frame.empty()) {
        SPDLOG_WARN("Empty");
        continue;
      }

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.params_ = buff_param_.TransformToDouble();
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 11);

      cv::imshow("new", frame);
      cv::imshow(window_handle_, frame);
      char key = cv::waitKey(3);
      if (key == 's' || key == 'S') {
        buff_param_.Write(param_path_);
      } else if (key == 'q' || key == 27 || key == 'Q') {
        cv::destroyAllWindows();
        return;
      }
    }
  }
};
