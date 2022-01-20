#include "app.hpp"
#include "armor_detector.hpp"
#include "armor_param.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

namespace {

const std::string kPARAM = "../../../../runtime/RMUL2021_Armor.json";
const std::string kWINDOW = "ui_setting";

}  // namespace

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
    cam_.Setup(480, 640);
    detector_.SetEnemyTeam(game::Team::kBLUE);
    armor_param_.Read(kPARAM);
  }

  ~UIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down UIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(kWINDOW, 1);

    cv::createTrackbar("binary_th", kWINDOW, &armor_param_.param_int.binary_th,
                       255, 0);
    cv::createTrackbar("contour_size_low_th", kWINDOW,
                       &armor_param_.param_int.contour_size_low_th, 50);
    cv::createTrackbar("contour_area_low_th", kWINDOW,
                       &armor_param_.param_int.contour_area_low_th, 200);
    cv::createTrackbar("contour_area_high_th", kWINDOW,
                       &armor_param_.param_int.contour_area_high_th, 200);
    cv::createTrackbar("bar_area_low_th", kWINDOW,
                       &armor_param_.param_int.bar_area_low_th, 200);
    cv::createTrackbar("bar_area_high_th", kWINDOW,
                       &armor_param_.param_int.bar_area_high_th, 200);
    cv::createTrackbar("angle_high_th", kWINDOW,
                       &armor_param_.param_int.angle_high_th, 1000);
    cv::createTrackbar("aspect_ratio_low_th", kWINDOW,
                       &armor_param_.param_int.aspect_ratio_low_th, 100);
    cv::createTrackbar("aspect_ratio_high_th", kWINDOW,
                       &armor_param_.param_int.aspect_ratio_high_th, 100);

    cv::createTrackbar("angle_diff_th", kWINDOW,
                       &armor_param_.param_int.angle_diff_th, 1000);
    cv::createTrackbar("length_diff_th", kWINDOW,
                       &armor_param_.param_int.length_diff_th, 1000);
    cv::createTrackbar("height_diff_th", kWINDOW,
                       &armor_param_.param_int.height_diff_th, 1000);
    cv::createTrackbar("area_diff_th", kWINDOW,
                       &armor_param_.param_int.area_diff_th, 1000);
    cv::createTrackbar("center_dist_low_th", kWINDOW,
                       &armor_param_.param_int.center_dist_low_th, 1000);
    cv::createTrackbar("center_dist_high_th", kWINDOW,
                       &armor_param_.param_int.center_dist_high_th, 255);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);

    while (true) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.params_ = armor_param_.transform2Double();
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 3);

      cv::imshow(kWINDOW, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's' || key == 'S') {
        armor_param_.Write(kPARAM);
      } else if (key == 'q' || key == 27 || key == 'Q') {
        cv::destroyAllWindows();
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
