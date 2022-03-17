#include "app.hpp"
#include "armor_detector.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "opencv2/opencv.hpp"
#include "radar_detector.hpp"
#include "robot.hpp"

class Radar : private App {
 private:
  Robot robot_;
  HikCamera cam_, base_cam_, outpost_cam_;
  RadarDetector detector_;

 public:
  Radar(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS2");
    cam_.Open(0);
    base_cam_.Open(1);
    outpost_cam_.Open(2);
    cam_.Setup(640, 480);
    base_cam_.Setup(640, 480);
    outpost_cam_.Setup(640, 480);
  }

  ~Radar() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");

    while (1) {
      cv::Mat frame[3];
      frame[0] = cam_.GetFrame();
      frame[1] = base_cam_.GetFrame();
      frame[2] = outpost_cam_.GetFrame();

      cv::Mat dst;
      cv::hconcat(frame, 3, dst);

      detector_.Detect(frame[0]);
      if (' ' == cv::waitKey(10)) {
        cv::waitKey(0);
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  Radar radar("logs/radar.log");
  radar.Run();
  return EXIT_SUCCESS;
}
