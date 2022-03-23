#include "demo.hpp"
#include "hik_camera.hpp"
#include "opencv2/opencv.hpp"

namespace {

const std::string kSOURCE = "../../../../../redbuff01.avi";
const std::string kOUTPUT = "../../../../../writer.avi";

}  // namespace

class CameraDemo : public App {
 private:
  cv::VideoWriter writer_;
  HikCamera cam_;
  bool recording_;

 public:
  CameraDemo(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */
    cam_.Open(0);
    cam_.Setup(640, 480);

    writer_.open(kOUTPUT, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60,
                 cv::Size(640, 480));
    if (!writer_.isOpened()) {
      SPDLOG_ERROR("Error occur");
    }

    recording_ = false;
  }

  ~CameraDemo() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Buff Aiming System. *****");
  }

  /* Demo Running */
  void Run() {
    SPDLOG_WARN("***** Running Buff Aiming System. *****");
    cv::Mat frame;

    while (1) {
      cam_.GetFrame(frame);
      if (frame.empty()) {
        SPDLOG_ERROR("GetFrame is null");
        continue;
      }
      if (recording_) {
        writer_.write(frame);
      }

      cv::imshow("RESULT", frame);
      int key = cv::waitKey(10);
      if ('q' == key || 'Q' == key) {
        break;
      }
      if ('s' == key || 'S' == key) {
        recording_ = true;
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  CameraDemo camera_demo("logs/camera_demo.log");
  camera_demo.Run();

  return EXIT_SUCCESS;
}
