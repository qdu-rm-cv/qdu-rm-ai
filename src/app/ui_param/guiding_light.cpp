#include "app.hpp"
#include "guidinglight_detector.hpp"
#include "guidinglight_param.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

namespace {

const std::string kPARAM = "../../../../runtime/RMUL2021_GuidingLight.json";
const std::string kWINDOW = "ui_setting";

}  // namespace

class UIParam : private App {
 private:
  HikCamera cam_;
  GuidingLightDetector detector_;
  GuidingLightParam guidinglight_param_;

 public:
  UIParam(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up UIParam System. *****");

    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(640, 480);
    // detector_.SetEnemyTeam(game::Team::kBLUE);
    guidinglight_param_.Read(kPARAM);
  }

  ~UIParam() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down UIParam System. *****");
  }

  /* 运行的主程序 */
  void Run() override {
    SPDLOG_WARN("Start UI Setting");
    cv::namedWindow(kWINDOW, 1);

    cv::createTrackbar("thresholdStep", kWINDOW,
                       &guidinglight_param_.param_int.thresholdStep, 40);
    cv::createTrackbar("minThreshold", kWINDOW,
                       &guidinglight_param_.param_int.minThreshold, 255);
    cv::createTrackbar("maxThreshold", kWINDOW,
                       &guidinglight_param_.param_int.maxThreshold, 500);
    cv::createTrackbar("minArea", kWINDOW,
                       &guidinglight_param_.param_int.minArea, 200);
    cv::createTrackbar("maxArea", kWINDOW,
                       &guidinglight_param_.param_int.maxArea, 5000);
    cv::createTrackbar("minCircularity", kWINDOW,
                       &guidinglight_param_.param_int.minCircularity, 20);
    cv::createTrackbar("maxCircularity", kWINDOW,
                       &guidinglight_param_.param_int.maxCircularity, 20);
    cv::createTrackbar("minInertiaRatio", kWINDOW,
                       &guidinglight_param_.param_int.minInertiaRatio, 20);
    cv::createTrackbar("maxInertiaRatio", kWINDOW,
                       &guidinglight_param_.param_int.maxInertiaRatio, 20);
    cv::createTrackbar("minConvexity", kWINDOW,
                       &guidinglight_param_.param_int.minConvexity, 20);
    cv::createTrackbar("maxConvexity", kWINDOW,
                       &guidinglight_param_.param_int.maxConvexity, 20);

    cv::Mat blank = cv::Mat::zeros(320, 240, CV_8UC1);

    while (true) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;

      SPDLOG_INFO("frame size {},{}", frame.size().width, frame.size().height);

      detector_.ResetByParam(guidinglight_param_.Transform2Double());
      detector_.Detect(frame);
      detector_.VisualizeResult(frame, 3);

      cv::imshow(kWINDOW, frame);
      cv::imshow("img", frame);
      char key = cv::waitKey(10);
      if (key == 's' || key == 'S') {
        guidinglight_param_.Write(kPARAM);
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