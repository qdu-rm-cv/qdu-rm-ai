#include "snipe_detector.hpp"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

namespace {

const auto kCV_FONT = cv::FONT_HERSHEY_SIMPLEX;
const cv::Scalar kGREEN(0., 255., 0.);
const cv::Scalar kRED(0., 0., 255.);
const cv::Scalar kYELLOW(0., 255., 255.);

}  // namespace

void SnipeDetector::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "a" << 0;
  fs << "b" << 0;
  SPDLOG_DEBUG("Inited params.");
}

bool SnipeDetector::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    params_.a = fs["a"];
    params_.b = fs["b"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void SnipeDetector::FindArmor(const cv::Mat &frame) { (void)frame; }

void SnipeDetector::VisualizeArmor(const cv::Mat &output, bool add_lable) {
  (void)output;
  (void)add_lable;
  // TODO : Realize
}

SnipeDetector::SnipeDetector() { SPDLOG_TRACE("Constructed."); }

SnipeDetector::SnipeDetector(const std::string &params_path,
                             game::Team enemy_team) {
  LoadParams(params_path);
  enemy_team_ = enemy_team;
  SPDLOG_TRACE("Constructed.");
}

SnipeDetector::~SnipeDetector() { SPDLOG_TRACE("Destructed."); }

void SnipeDetector::SetEnemyTeam(game::Team enemy_team) {
  enemy_team_ = enemy_team;
  SPDLOG_DEBUG("{}", game::TeamToString(enemy_team));
}

const tbb::concurrent_vector<Armor> &SnipeDetector::Detect(
    const cv::Mat &frame) {
  (void)frame;
  targets_.clear();
  // TODO : Realize
  targets_.emplace_back(Armor());
  return targets_;
}

void SnipeDetector::VisualizeResult(const cv::Mat &output, int verbose) {
  VisualizeArmor(output, verbose > 2);
}
