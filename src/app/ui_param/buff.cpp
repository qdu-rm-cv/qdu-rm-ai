#include "app.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"
#include "buff_param.hpp"
#include "buff_detector.hpp"

namespace {

const std::string kPARAM = "../../../../runtime/RMUT2021_Buff.json";
const std::string kWINDOW = "ui_setting";

}  // namespace

class UIParam : private App {
 private:
  HikCamera cam_;
  BuffDetector detector_;
  BuffParam buff_param_;

 public:
  UIParam(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up UIParam System. *****");

    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(640, 480);
    buff_param_.Read(kPARAM);
    detector_.SetTeam(game::Team::kRED);
  }

  ~UIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down UIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(kWINDOW, 1);

    cv::createTrackbar("binary_th", kWINDOW, &buff_param_.param_int.binary_th,
                       255, 0);
    cv::createTrackbar("contour_size_low_th", kWINDOW,
                       &buff_param_.param_int.contour_size_low_th, 50);
    cv::createTrackbar("contour_center_area_low_th", kWINDOW,
                       &buff_param_.param_int.contour_center_area_low_th, 3000);
    cv::createTrackbar("contour_center_area_high_th", kWINDOW,
                       &buff_param_.param_int.contour_center_area_high_th, 6000);
    cv::createTrackbar("rect_center_ratio_low_th", kWINDOW,
                       &buff_param_.param_int.rect_center_ratio_low_th, 3000);
    cv::createTrackbar("rect_center_ratio_high_th", kWINDOW,
                       &buff_param_.param_int.rect_center_ratio_high_th, 3000);
    cv::createTrackbar("rect_ratio_low_th", kWINDOW,
                       &buff_param_.param_int.rect_ratio_low_th, 3000);
    cv::createTrackbar("rect_ratio_high_th", kWINDOW,
                       &buff_param_.param_int.rect_ratio_high_th, 6000);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);

    while (true) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.params_ = buff_param_.Transform2Double();
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 11);

      cv::imshow(kWINDOW, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(3);
      if (key == 's' || key == 'S') {
        buff_param_.Write(kPARAM);
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
