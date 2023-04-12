#pragma once

#include "app.hpp"
#include "hik_camera.hpp"
#include "opencv2/opencv.hpp"

class UI : private App {
 public:
  HikCamera cam_;
  std::string param_path_, window_handle_;

  UI(const std::string& log_path, const std::string& param_path,
     const std::string& window = "ui_setting")
      : App(log_path), param_path_(param_path), window_handle_(window) {
    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(kIMAGE_WIDTH, kIMAGE_HEIGHT);
  }
  ~UI() {}

  virtual void Run() = 0;
};
