#include "app.hpp"
#include "buff_detector.hpp"
#include "compensator.hpp"
#include "hik_camera.hpp"
#include "opencv2/opencv.hpp"
#include "buff_predictor.hpp"
#include "robot.hpp"

class BuffAim : private App {
 private:
  Robot robot_;
  HikCamera cam_;
  BuffDetector detector_;
  BuffPredictor predictor_;
  Compensator compensator_;

 public:
  BuffAim(const std::string& log_path) : App(log_path) {
    SPDLOG_WARN("***** Setting Up Buff Aiming System. *****");

    /* 初始化设备 */
    robot_.Init("/dev/ttyTHS2");
    cam_.Open(1);
    cam_.Setup(640, 480);
    detector_.LoadParams("RMUT2021_Buff.json");
    compensator_.LoadCameraMat("MV-CA016-10UC-6mm.json");

    do {
      predictor_.SetTime(robot_.GetTime());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while ((robot_.GetEnemyTeam() != game::Team::kUNKNOWN) &&
             (predictor_.GetTime() != 0));

    detector_.SetTeam(robot_.GetEnemyTeam());
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
      predictor_.SetBuff(detector_.Detect(frame).back());
      predictor_.Predict();

      detector_.VisualizeResult(frame, 5);
      predictor_.VisualizePrediction(frame, true);
      cv::imshow("win", frame);
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
