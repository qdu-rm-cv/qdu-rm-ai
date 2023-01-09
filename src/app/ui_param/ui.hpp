// #pragma once

// #include "app.hpp"
// #include "detector.hpp"
// #include "hik_camera.hpp"
// #include "opencv2/opencv.hpp"
// #include "param.hpp"

// template <typename D, typename P>
// class UI {
//  private:
//   HikCamera cam_;
//   Detector detector_;
//   Param param_;
//   std::string param_path_, window_handle_;

//  public:
//   UI(const std::string& log_path, const std::string& param_path,
//      const std::string& window = "ui_setting")
//       : App(log_path), param_path_(param_path), window_handle_(window) {
//     SPDLOG_WARN("***** Setting Up BuffUIParam System. *****");

//     /* 初始化设备 */
//     cam_.Open(0);
//     cam_.Setup(640, 480);
//     param_.Read(param_path_);
//     detector_.SetTeam(game::Team::kRED);
//   }
//   ~UI();
// };
