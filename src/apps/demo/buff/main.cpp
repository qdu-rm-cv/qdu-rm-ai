#include "buff_detector.hpp"
#include "buff_predictor.hpp"
#include "demo.hpp"
#include "opencv2/opencv.hpp"

namespace {

const std::string kSOURCE = "../../../../../../redbuff01.avi";
const std::string kOUTPUT = "../../../../../../writer.avi";

}  // namespace

class BuffDemo : public Demo {
 private:
  BuffDetector detector_;
  BuffPredictor predictor_;

 public:
  explicit BuffDemo(const std::string& log_path) : Demo(log_path) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */

    detector_.LoadParams(kPATH_RUNTIME + "RMUT2021_Buff.json");
    predictor_.LoadParams(kPATH_RUNTIME + "RMUT2022_Buff_Pre.json");

    detector_.SetTeam(game::Team::kBLUE);
    predictor_.SetRace(game::Race::kRMUC);
    predictor_.SetTime(10);
  }

  ~BuffDemo() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Buff Aiming System. *****");
  }

  /* Demo Running */
  void Run() {
    SPDLOG_WARN("***** Running Buff Aiming System. *****");

    while (1) {
      cv::Mat frame = Read();
      if (frame.empty()) {
        SPDLOG_ERROR("GetFrame is null");
        continue;
      }

      auto buffs = detector_.Detect(frame);

      if (buffs.size() > 0) {
        predictor_.SetBuff(buffs.back());
        auto armors = predictor_.Predict();
        if (armors.size() != 0) {
          predictor_.VisualizePrediction(frame, 10);
        }
        detector_.VisualizeResult(frame, 10);
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

  BuffDemo buff_demo("logs/buff_demo.log");
  buff_demo.Open(kSOURCE, kOUTPUT);
  buff_demo.Run();

  return EXIT_SUCCESS;
}
