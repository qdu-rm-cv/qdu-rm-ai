#include "aim_assitant.hpp"
#include "app.hpp"
#include "behavior.hpp"
#include "common.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "robot.hpp"

class AutomaticAim : public App {
 private:
  Robot robot_;
  HikCamera cam_;
  AimAssitant assitant_;
  Compensator compensator_;
  Behavior manager_;

  game::Arm arm_ = game::Arm::kUNKNOWN;

 public:
  explicit AutomaticAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");
    cam_.Open(0);
    cam_.Setup(480, 640);

    assitant_.LoadParams("../../../../runtime/RMUL2022_Armor.json",
                         "../../../../runtime/RMUT2022_Buff.json",
                         "../../../../runtime/RMUT2022_Snipe.json",
                         "../../../../runtime/RMUT2022_Armor_Pre.json",
                         "../../../../runtime/RMUT2022_Buff_Pre.json",
                         "../../../../runtime/RMUT2022_AntiTop.json");
    assitant_.SetClassiferParam(
        "../../../../runtime/armor_classifier.onnx",
        "../../../../runtime/armor_classifier_lable.json", cv::Size(28, 28));

    compensator_.LoadCameraMat("runtime/MV-CA016-10UC-6mm.json");
  }

  void Init() {
    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS2");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    arm_ = robot_.GetArm();
    assitant_.SetEnemyTeam(robot_.GetEnemyTeam());
    assitant_.SetRace(robot_.GetRace());
    assitant_.SetArm(arm_);
    assitant_.SetTime(robot_.GetTime());
    manager_.Update(robot_.GetBaseHP(), robot_.GetSentryHP(),
                    robot_.GetBalletRemain());
  }

  void TestInit() {
    robot_.Init("/dev/ttyUSB0");

    arm_ = game::Arm::kSENTRY;
    assitant_.SetEnemyTeam(game::Team::kBLUE);
    assitant_.SetRace(game::Race::kRMUC);
    assitant_.SetArm(game::Arm::kSENTRY);
    assitant_.SetTime(10);
    manager_.Update(robot_.GetBaseHP(), robot_.GetSentryHP(),
                    robot_.GetBalletRemain());
  }

  ~AutomaticAim() {
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

      assitant_.SetRFID(robot_.GetRFID());
      auto armors = assitant_.Aim(frame);

      if (armors.size() > 0) {
        compensator_.Apply(armors, frame, robot_.GetBalletSpeed(),
                           robot_.GetEuler(), assitant_.GetMethod());
        Armor armor = armors.front();

        if (arm_ == game::Arm::kSENTRY) {
          manager_.Update(robot_.GetBaseHP(), robot_.GetSentryHP(),
                          robot_.GetBalletRemain());
          manager_.Move(robot_.GetChassicSpeed());
        }

        manager_.Aim(armor.GetAimEuler());
        assitant_.VisualizeResult(frame, 10);
        robot_.Pack(manager_.GetData(), armor.GetTransVec().at<double>(0, 2));
      }

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

  AutomaticAim aim_assitant("logs/aim_assitant.log");
  aim_assitant.TestInit();
  aim_assitant.Run();

  return EXIT_SUCCESS;
}
