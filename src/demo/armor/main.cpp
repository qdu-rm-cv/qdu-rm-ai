#include <iostream>

#include "app/app.hpp"
#include "armor_detector.hpp"
#include "hik_camera.hpp"
#include "opencv2/videoio.hpp"

namespace {

const std::string kSOURCE_PATH = "../../../../../../1.avi";
const std::string kOUTPUT_PATH = "../../../../image/test_detect.avi";
const std::string kPARAM_PATH = "../../../../runtime/RMUL2021_Armor.json ";

}  // namespace

class ArmorDemo : private App {
 private:
  cv::VideoCapture cap_;
  cv::VideoWriter writer_;
  ArmorDetector detector_;
  cv::Mat frame_;

 public:
  ArmorDemo(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Armor Detecting Demo. *****");

    /* 初始化 */
    cap_.open(kSOURCE_PATH);

    const int codec = cap_.get(cv::CAP_PROP_FOURCC);
    const double fps = cap_.get(cv::CAP_PROP_FPS);
    cv::Size f_size(cap_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    writer_.open(kOUTPUT_PATH, codec, fps, f_size);

    if (!cap_.isOpened()) {
      SPDLOG_ERROR("Can't Open Video {}", kSOURCE_PATH);
    }
    if (!writer_.isOpened()) {
      SPDLOG_ERROR("Can't Write Video {}", kOUTPUT_PATH);
    }

    detector_.LoadParams(kPARAM_PATH);
    detector_.SetEnemyTeam((game::Team::kBLUE));
  }

  ~ArmorDemo() { SPDLOG_WARN("***** Shuted Down Armor Detecting Demo. *****"); }

  /* Demo Running */
  void Run() {
    SPDLOG_WARN("***** Running Armor Detecting Demo. *****");

    const int delay = static_cast<int>(1000. / cap_.get(cv::CAP_PROP_FPS));

    while (1) {
      cap_.read(frame_);
      if (frame_.empty()) continue;

      auto armors = detector_.Detect(frame_);
      detector_.VisualizeResult(frame_, 10);
      writer_.write(frame_);

      cv::imshow("video", frame_);
      if (' ' == cv::waitKey(delay)) {
        cv::waitKey(0);
      }
      if ('q' == cv::waitKey(delay)) {
        exit(0);
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  ArmorDemo demo("logs/armor_demo.log");
  demo.Run();

  return 0;
}
