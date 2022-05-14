#define async
#ifdef async

#include "app.hpp"
#include "async_armor_detector.hpp"
#include "behavior.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class AutoAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  ArmorDetectorAsync detector_async_;
  Compensator compensator_;
  Behavior manager_;

  component::Recorder recorder_ = component::Recorder("AutoAimThread");

 public:
  AutoAim(const std::string& log_path)
      : App(log_path), detector_async_(ArmorDetectorAsync(2)) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyACM0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_async_.LoadParams("../../../../runtime/RMUL2022_Armor.json");
    compensator_.LoadCameraMat("../../../../runtime/MV-CA016-10UC-6mm_1.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    detector_async_.SetEnemyTeam(robot_.GetEnemyTeam());
    // detector_async_.SetEnemyTeam(game::Team::kBLUE);
  }

  ~AutoAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");
    cv::Mat frame;
    tbb::concurrent_vector<Armor> armors;
    detector_async_.Start();

    while (1) {
      if (!cam_.GetFrame(frame)) continue;

      detector_async_.PutFrame(frame);
      if (!detector_async_.GetResult(armors)) continue;

      compensator_.Apply(armors, frame, robot_.GetBalletSpeed(),
                         robot_.GetEuler(), component::AimMethod::kARMOR);
      manager_.Aim(armors.front().GetAimEuler());
      robot_.Pack(manager_.GetData(), 9999);
      SPDLOG_WARN("pack");
      recorder_.Record();
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

#else

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

  component::Recorder recorder_ = component::Recorder("AutoAimThread");

 public:
  AutoAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init();
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_.LoadParams("../../../../runtime/RMUL2022_Armor.json");
    compensator_.LoadCameraMat("../../../../runtime/MV-CA016-10UC-6mm_1.json");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    detector_.SetEnemyTeam(robot_.GetEnemyTeam());
    // detector_.SetEnemyTeam(game::Team::kBLUE);
  }

  ~AutoAim() {
    /* 关闭设备 */

    SPDLOG_WARN("***** Shuted Down Auto Aiming System. *****");
  }

  /* 运行的主程序 */
  void Run() {
    SPDLOG_WARN("***** Running Auto Aiming System. *****");
    cv::Mat frame;

    while (1) {
      cam_.GetFrame(frame);
      if (frame.empty()) continue;
      auto armors = detector_.Detect(frame);

      if (armors.size() != 0) {
        compensator_.Apply(armors, frame, robot_.GetBalletSpeed(),
                           robot_.GetEuler());
        manager_.Aim(armors.front().GetAimEuler());
        robot_.Pack(manager_.GetData(), 9999);

        detector_.VisualizeResult(frame, 10);
      }
      cv::imshow("show", frame);
      if (' ' == cv::waitKey(10)) {
        cv::waitKey(0);
      }
      recorder_.Record();
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

#endif
