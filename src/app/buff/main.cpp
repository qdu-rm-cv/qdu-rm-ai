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
  explicit BuffAim(const std::string& log_path)
      : App(log_path, component::Logger::FMT::kFMT_THREAD) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyACM0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_.LoadParams("../../../../runtime/RMUT2022_Buff.json");
    predictor_.LoadParams("../../../../runtime/RMUT2022_Buff_Pre.json");
    compensator_.LoadCameraMat("../../../../runtime/MV-CA016-10UC-6mm_2.json");

    // do {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // } while ((robot_.GetEnemyTeam() != game::Team::kUNKNOWN) &&
    //          (robot_.GetTime() != 0) &&
    //          (robot_.GetRace() != game::Race::kUNKNOWN));
    // detector_.SetTeam(robot_.GetEnemyTeam());
    // predictor_.SetTime(robot_.GetTime());
    // predictor_.SetRace(robot_.GetRace());
    detector_.SetTeam(game::Team::kRED);
    predictor_.SetRace(game::Race::kRMUT);
    predictor_.SetTime(10);
  }

  ~BuffAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Buff Aiming System. *****");
  }

  void Run() {
    SPDLOG_WARN("***** Running Buff Aiming System. *****");
    cv::Mat frame;

    while (1) {
      if (!cam_.GetFrame(frame)) continue;

      auto buffs = detector_.Detect(frame);
      if (buffs.size() > 0) {
        predictor_.SetBuff(buffs.back());
        auto armors = predictor_.Predict();
        auto armor = armors.front();
        SPDLOG_WARN("size : {}", armors.size());
        // auto armor = buffs.front().GetTarget();
        compensator_.Apply(armor, frame, robot_.GetBalletSpeed(),
                           robot_.GetEuler(), component::AimMethod::kBUFF);
        manager_.Aim(armor.GetAimEuler());
        robot_.Pack(manager_.GetData(), 9999);

        predictor_.VisualizePrediction(frame, 10);

        detector_.VisualizeResult(frame, 10);
      }

      cv::imshow("frame", frame);
      char key = cv::waitKey(10);
      if (' ' == key) {
        cv::waitKey(0);
      } else if ('M' == key) {
        predictor_.ChangeDirection(true);
      }
      predictor_.ChangeDirection(robot_.GetNotice());
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
