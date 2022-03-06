#pragma once

#include "app.hpp"
#include "guidinglight_detector.hpp"
#include "guidinglight_param.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class LightUIParam : private App {
 private:
  HikCamera cam_;
  GuidingLightDetector detector_;
  GuidingLightParam guidinglight_param_;
  std::string param_path_, window_handle_;

 public:
  LightUIParam(const std::string& log_path, const std::string& param_path,
               const std::string& window = "ui_setting")
      : App(log_path), param_path_(param_path), window_handle_(window) {
    SPDLOG_WARN("***** Setting Up LightUIParam System. *****");

    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(640, 480);
    guidinglight_param_.Read(param_path_);
  }

  ~LightUIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down LightUIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(window_handle_, 1);

    cv::createTrackbar("thresholdStep", window_handle_,
                       &guidinglight_param_.parami_.thresholdStep, 40);
    cv::createTrackbar("minThreshold", window_handle_,
                       &guidinglight_param_.parami_.minThreshold, 255);
    cv::createTrackbar("maxThreshold", window_handle_,
                       &guidinglight_param_.parami_.maxThreshold, 500);
    cv::createTrackbar("minArea", window_handle_,
                       &guidinglight_param_.parami_.minArea, 200);
    cv::createTrackbar("maxArea", window_handle_,
                       &guidinglight_param_.parami_.maxArea, 5000);
    cv::createTrackbar("minCircularity", window_handle_,
                       &guidinglight_param_.parami_.minCircularity, 20);
    cv::createTrackbar("maxCircularity", window_handle_,
                       &guidinglight_param_.parami_.maxCircularity, 20);
    cv::createTrackbar("minInertiaRatio", window_handle_,
                       &guidinglight_param_.parami_.minInertiaRatio, 20);
    cv::createTrackbar("maxInertiaRatio", window_handle_,
                       &guidinglight_param_.parami_.maxInertiaRatio, 20);
    cv::createTrackbar("minConvexity", window_handle_,
                       &guidinglight_param_.parami_.minConvexity, 20);
    cv::createTrackbar("maxConvexity", window_handle_,
                       &guidinglight_param_.parami_.maxConvexity, 20);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);

    while (true) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.ResetByParam(guidinglight_param_.TransformToDouble());
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 3);

      cv::imshow(window_handle_, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's' || key == 'S') {
        guidinglight_param_.Write(param_path_);
      } else if (key == 'q' || key == 27 || key == 'Q') {
        cv::destroyAllWindows();
        return;
      }
    }
  }
};
