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

 public:
  AutomaticAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Auto Aiming System. *****");
    cam_.Open(0);
    cam_.Setup(480, 640);

    assitant_.LoadParams(
        "/runtime/RMUL2021_Armor.json", "/runtime/RMUT2021_Buff.json",
        "/runtime/RMUT2022_Snipe.json", "/runtime/RMUT2022_Armor_Pre.json",
        "/runtime/RMUT2022_Buff_Pre.json");
    assitant_.SetClassiferParam("../../../runtime/armor_classifier.onnx",
                                "../../../runtime/armor_classifier_lable.json",
                                cv::Size(28, 28));

    compensator_.LoadCameraMat("runtime/MV-CA016-10UC-6mm.json");
  }

  void Init() {
    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS2");

    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (robot_.GetEnemyTeam() != game::Team::kUNKNOWN);

    assitant_.SetEnemyTeam(robot_.GetEnemyTeam());
    assitant_.SetRace(robot_.GetRace());
    assitant_.SetArm(robot_.GetArm());
    assitant_.SetTime(robot_.GetTime());
    manager_.Update(robot_.GetBaseHP(), robot_.GetSentryHP(),
                    robot_.GetBalletRemain());
  }

  void TestInit() {
    robot_.Init("/dev/ttyUSB0");

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

    while (1) {
      cv::Mat frame = cam_.GetFrame();
      if (frame.empty()) continue;

      assitant_.SetRFID(robot_.GetRFID());
      auto armors = assitant_.Aim(frame);
      compensator_.Apply(armors, frame, robot_.GetRotMat());
      Armor armor = armors.front();
      assitant_.VisualizeResult(frame, 10);

      manager_.Aim(armor.GetAimEuler());
      if (robot_.GetArm() == game::Arm::kSENTRY)
        manager_.Move(robot_.GetChassicSpeed());
      robot_.Pack(manager_.GetData(), armor.GetTransVec().at<double>(0, 2));

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
