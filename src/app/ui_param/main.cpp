#include "app.hpp"
#include "armor_detector.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"
#include "armor_param.hpp"

namespace {
  const std::string kPARAM = "../../../../runtime/RMUL2021_Armor.json";
  const std::string kWINDOW = "ui_setting";
}

class UIParam : private App {
 private:
  HikCamera cam_;
  ArmorDetector detector_;
  ArmorParam armor_param_;

 public:
  UIParam(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up UIParam System. *****");

    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(640, 480);
    armor_param_.Read(kPARAM);
  }

  ~UIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down UIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("start_UiSetting");
    cv::namedWindow(kWINDOW, 1);

    cv::createTrackbar("binary_th", kWINDOW, &armor_param_.param_int.binary_th, 255, 0);
    cv::createTrackbar("contour_size_low_th", kWINDOW,
                       &armor_param_.param_int.contour_size_low_th, 10240);
    cv::createTrackbar("contour_area_low_th", kWINDOW,
                       &armor_param_.param_int.contour_area_low_th, 255);
    cv::createTrackbar("contour_area_high_th", kWINDOW,
                       &armor_param_.param_int.contour_area_high_th, 255);
    cv::createTrackbar("bar_area_low_th", kWINDOW, &armor_param_.param_int.bar_area_low_th,
                       255);
    cv::createTrackbar("bar_area_high_th", kWINDOW,
                       &armor_param_.param_int.bar_area_high_th, 255);
    cv::createTrackbar("angle_high_th", kWINDOW, &armor_param_.param_int.angle_high_th,
                       255);
    cv::createTrackbar("aspect_ratio_low_th", kWINDOW,
                       &armor_param_.param_int.aspect_ratio_low_th, 255);
    cv::createTrackbar("aspect_ratio_high_th", kWINDOW,
                       &armor_param_.param_int.aspect_ratio_high_th, 255);
    cv::createTrackbar("angle_diff_th", kWINDOW, &armor_param_.param_int.angle_diff_th,
                       255);
    cv::createTrackbar("length_diff_th", kWINDOW, &armor_param_.param_int.length_diff_th,
                       255);
    cv::createTrackbar("height_diff_th", kWINDOW, &armor_param_.param_int.height_diff_th,
                       255);
    cv::createTrackbar("area_diff_th", kWINDOW, &armor_param_.param_int.area_diff_th, 255);
    cv::createTrackbar("center_dist_low_th", kWINDOW,
                       &armor_param_.param_int.center_dist_low_th, 255);
    cv::createTrackbar("center_dist_high_th", kWINDOW,
                       &armor_param_.param_int.center_dist_high_th, 255);

    while (true) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty())  continue;

      detector_.params_ = armor_param_.transform2Double();
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 10);
      cv::imshow(kWINDOW, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's') {
        armor_param_.Write(kPARAM);
      } else if (key == 'q' || key == 27) {
        return;
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  UIParam ui_param("logs/ui_param.log");
  ui_param.Run();

  return EXIT_SUCCESS;
}
