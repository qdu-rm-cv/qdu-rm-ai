#include "app.hpp"
#include "armor_detector.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class Radar : private App {
 private:
  Robot robot_;
  HikCamera cam_, base_cam_, outpost_cam_;
  ArmorDetector detector_;
  Compensator compensator_;

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

    detector_.LoadParams("RMUL2021_Armor.json");
    compensator_.LoadCameraMat("MV-CA016-10UC-6mm.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    detector_.SetEnemyTeam(robot_.GetEnemyTeam());
  }

  ~Radar() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");

    while (1) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;
      auto armors = detector_.Detect(frame);
      // target = predictor.Predict(armors, frame);
      // compensator_.Apply(target, frame, robot_.GetRotMat());
      // robot_.Aim(target.GetAimEuler(), false);
      detector_.VisualizeResult(frame, 10);
      cv::imshow("show", frame);
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
