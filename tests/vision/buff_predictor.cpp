
#include "buff_predictor.hpp"

#include "armor_detector.hpp"
#include "buff_detector.hpp"
#include "buff_predictor.hpp"
#include "gtest/gtest.h"
#include "opencv2/opencv.hpp"

namespace {

const std::string kPARAM_DETECT("../../../runtime/RMUT2021_Buff.json");
const std::string kPARAM_PREDICT("../../../runtime/RMUT2022_Buff_Pre.json");

}  // namespace

TEST(TestVision, TestKalmanPredictor) {
  RMlogger::SetLogger();
  cv::Mat frame;
  BuffDetector detector(kPARAM_DETECT, game::Team::kBLUE);
  BuffPredictor predictor(kPARAM_PREDICT);
  predictor.SetRace(game::Race::kRMUT);
  predictor.SetTime(10);

  cv::VideoCapture cap("../../../../redbuff01.avi");
  if (!cap.isOpened()) {
    SPDLOG_WARN("Video can't be loaded.");
    return;
  }

  SPDLOG_WARN("[Start Test]");
  cap >> frame;
  while (!frame.empty()) {
    cap >> frame;

    auto buffs = detector.Detect(frame);
    predictor.SetBuff(buffs.front());
    predictor.Predict();
    detector.VisualizeResult(frame, 3);
    predictor.VisualizePrediction(frame, 3);

    cv::imshow("Predictor", frame);
    auto key = cv::waitKey(10);
    if (key == 'q') {
      cv::waitKey(0);
      cap.release();
      return;
    } else if (key == ' ')
      cv::waitKey(0);
  }
  cv::destroyAllWindows();
  cap.release();
}
