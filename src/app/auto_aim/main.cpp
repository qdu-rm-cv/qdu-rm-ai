#include "app.hpp"
#include "armor_detector.hpp"
#include "behavior.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class AutoAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  ArmorDetector detector_;
  Compensator compensator_;
  Behavior manager_;

 public:
  AutoAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_.LoadParams("../../../../runtime/RMUL2022_Armor.json");
    compensator_.LoadCameraMat("../../../../runtime/MV-CA016-10UC-6mm.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    detector_.SetEnemyTeam(robot_.GetEnemyTeam());
  }

  ~AutoAim() {
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

      if (armors.size() == 0) continue;
      compensator_.Apply(armors, frame, robot_.GetEuler());
      manager_.Aim(armors.front().GetAimEuler());
      robot_.Pack(manager_.GetData(), 9999);
      
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

  AutoAim auto_aim("logs/auto_aim.log");
  auto_aim.Run();

  return EXIT_SUCCESS;
}
