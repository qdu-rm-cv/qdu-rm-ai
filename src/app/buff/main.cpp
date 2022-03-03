#include "app.hpp"
#include "behavior.hpp"
#include "buff_detector.hpp"
#include "buff_predictor.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "opencv2/opencv.hpp"
#include "robot.hpp"

class BuffAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  BuffDetector detector_;
  BuffPredictor predictor_;
  Compensator compensator_;
  Behavior manager_;

 public:
  BuffAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_.LoadParams("../../../../runtime/RMUT2021_Buff.json");
    predictor_.LoadParams("../../../../runtime/RMUT2022_Buff_Pre.json");
    compensator_.LoadCameraMat("../../../../runtime/MV-CA016-10UC-6mm_1.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while ((robot_.GetEnemyTeam() != game::Team::kUNKNOWN) &&
             (robot_.GetTime() != 0) &&
             (robot_.GetRace() != game::Race::kUNKNOWN));
    detector_.SetTeam(robot_.GetEnemyTeam());
    predictor_.SetTime(robot_.GetTime());
    predictor_.SetRace(robot_.GetRace());
    // detector_.SetTeam(game::Team::kRED);
    // predictor_.SetRace(game::Race::kRMUC);
    // predictor_.SetTime(10);
  }

  ~BuffAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Buff Aiming System. *****");
  }

  void Run() {
    SPDLOG_WARN("***** Running Buff Aiming System. *****");

    while (1) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) {
        SPDLOG_ERROR("cam.GetFrame is null");
        continue;
      }

      auto buffs = detector_.Detect(frame);

      if (buffs.size() > 0) {
        predictor_.SetBuff(buffs.back());
        auto armors = predictor_.Predict();
        if (armors.size() != 0) {
          compensator_.Apply(armors, frame, robot_.GetEuler());
          manager_.Aim(armors.front().GetAimEuler());
          robot_.Pack(manager_.GetData(), 9999);

          predictor_.VisualizePrediction(frame, 10);
        }

        detector_.VisualizeResult(frame, 10);
      }

      cv::imshow("xxx", frame);
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

  BuffAim buff_aim("logs/buff_aim.log");
  buff_aim.Run();

  return EXIT_SUCCESS;
}
