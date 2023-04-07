// #define async
#ifdef async

#include "app.hpp"
#include "armor_classifier.hpp"
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
  // ArmorClassifier classifier_;

 public:
  explicit AutoAim(const std::string& log_path)
      : App(log_path), detector_async_(ArmorDetectorAsync(2)) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyUSB0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_async_.LoadParams(kPATH_RUNTIME + "RMUL2022_Armor.json");
    compensator_.LoadCameraMat(kPATH_RUNTIME + "MV-CA016-10UC-6mm_2.json");
    // classifier_.LoadModel(kPATH_RUNTIME + "armor_classifier.onnx");
    // classifier_.LoadLable(kPATH_RUNTIME + "armor_classifier_lable.json");
    // classifier_.SetInputSize(cv::Size(28, 28));

    // do {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    // detector_async_.SetEnemyTeam(robot_.GetEnemyTeam());
    detector_async_.SetEnemyTeam(game::Team::kBLUE);
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
      robot_.GetEuler();
      detector_async_.PutFrame(frame);
      if (!detector_async_.GetResult(armors)) continue;

      // for (auto armor : armors) {
      //   classifier_.ClassifyModel(armor, frame);
      // }
      compensator_.Apply(armors.front(), frame, robot_.GetBalletSpeed(),
                         robot_.GetEuler(), game::AimMethod::kARMOR);
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
  explicit AutoAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyUSB0");
    cam_.Open(0);
    cam_.Setup(640, 480);
    detector_.LoadParams(kPATH_RUNTIME + "RMUL2022_Armor.json");
    compensator_.LoadCameraMat(kPATH_RUNTIME + "MV-CA016-10UC-6mm_2.json");

    // do {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    // detector_.SetEnemyTeam(robot_.GetEnemyTeam());
    detector_.SetEnemyTeam(game::Team::kBLUE);
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
      robot_.GetEuler();
      cam_.GetFrame(frame);
      if (frame.empty()) continue;
      /* cv::flip(frame, frame, -1); */ /* 平衡步兵 */
      auto armors = detector_.Detect(frame);
      if (armors.size() != 0) {
        compensator_.Apply(armors, frame, robot_.GetBalletSpeed(),
                           robot_.GetEuler(), game::AimMethod::kARMOR);
        manager_.Aim(armors.front().GetAimEuler());
        robot_.Pack(manager_.GetData(), 9999);

        detector_.VisualizeResult(frame, 10);
      }
      cv::line(frame, cv::Point2f(320, 0), cv::Point2f(320, 480),
               cv::Scalar(0, 0, 255), 2);
      cv::line(frame, cv::Point2f(0, 240), cv::Point2f(640, 240),
               cv::Scalar(0, 0, 255), 2);
      /* cv::flip(frame, frame, -1); */ /* 平衡步兵 */
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
