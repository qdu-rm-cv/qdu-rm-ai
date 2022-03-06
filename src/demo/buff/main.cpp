// #include "app/app.hpp"
#include "buff_detector.hpp"
#include "buff_predictor.hpp"
#include "demo.hpp"
#include "opencv2/opencv.hpp"
#include "raspi_camera.hpp"

namespace {

const std::string kSOURCE = "../../../../../redbuff01.avi";
const std::string kOUTPUT = "../../../../../writer.avi";

}  // namespace

class BuffDemo : public Demo {
 private:
  BuffDetector detector_;
  BuffPredictor predictor_;

 public:
  BuffDemo(const std::string& log_path) : Demo(log_path) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */

    detector_.LoadParams("../../../../runtime/RMUT2021_Buff.json");
    predictor_.LoadParams("../../../../runtime/RMUT2022_Buff_Pre.json");

    detector_.SetTeam(game::Team::kBLUE);
    predictor_.SetRace(game::Race::kRMUC);
    predictor_.SetTime(10);
  }

  ~BuffDemo() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Buff Aiming System. *****");
  }

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
        if (armors.size() != 0) predictor_.VisualizePrediction(frame, 10);
        detector_.VisualizeResult(frame, 10);
      }

      cv::imshow("RESULT", frame);
      cv::waitKey(1);
      if (' ' == cv::waitKey(10)) {
        cv::waitKey(0);
      }
    }
  }
};

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  BuffDemo buff_aim("logs/buff_aim.log");
  buff_aim.Open(kSOURCE, kOUTPUT);
  buff_aim.Run();

  return EXIT_SUCCESS;
}
