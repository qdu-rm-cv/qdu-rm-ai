#include "armor_detector.hpp"
#include "demo.hpp"
#include "opencv2/videoio.hpp"

namespace {

const std::string kSOURCE_PATH = "../../../../../../1.avi";
const std::string kOUTPUT_PATH = "../../../../image/test_detect.avi";
const std::string kPARAM_PATH = "../../../../runtime/RMUL2021_Armor.json ";

}  // namespace

class ArmorDemo : public Demo {
 private:
  ArmorDetector detector_;

 public:
  explicit ArmorDemo(const std::string& log_path) : Demo(log_path) {
    SPDLOG_WARN("***** Setting Up Armor Detecting Demo. *****");

    /* 初始化 */
    detector_.LoadParams(kPARAM_PATH);
    detector_.SetEnemyTeam(game::Team::kBLUE);
  }

  ~ArmorDemo() {
    /* 关闭 */

    SPDLOG_WARN("***** Shuted Down Armor Detecting Demo. *****");
  }

  /* Demo Running */
  void Run() {
    SPDLOG_WARN("***** Running Armor Detecting Demo. *****");

    while (1) {
      cv::Mat frame = Read();
      if (frame.empty()) {
        SPDLOG_ERROR("GetFrame is null");
        continue;
      }

      auto armors = detector_.Detect(frame);
      if (armors.size() > 0) {
        detector_.VisualizeResult(frame, 10);
        Write(frame);
      }

      cv::imshow("RESULT", frame);
      if ('q' == cv::waitKey(10)) {
        break;
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  ArmorDemo demo("logs/armor_demo.log");
  demo.Open(kSOURCE_PATH, kOUTPUT_PATH);
  demo.Run();

  return 0;
}
