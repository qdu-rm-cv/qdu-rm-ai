#include "snipe_detector.hpp"

#include <execution>

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

void SnipeDetector::FindArmor(const cv::Mat &frame) {
  (void)frame;
  // TODO :Realize
  // Formed an armor then please use method `SetModel(game::Model::kOUTPOST);`
  targets_.emplace_back(Armor());
}

void SnipeDetector::VisualizeArmor(const cv::Mat &output, bool add_lable) {
  auto draw_armor = [&](const auto &armor) {
    auto vertices = armor.ImageVertices();
    auto num_vertices = vertices.size();
    for (std::size_t i = 0; i < num_vertices; ++i) {
      cv::line(output, vertices[i], vertices[(i + 1) % num_vertices], kGREEN);
    }
    cv::drawMarker(output, armor.ImageCenter(), kGREEN, cv::MARKER_DIAMOND);

    if (add_lable) {
      cv::putText(output,
                  cv::format("%.2f, %.2f", armor.ImageCenter().x,
                             armor.ImageCenter().y),
                  vertices[1], kCV_FONT, 1.0, kGREEN);
    }
  };
  if (!targets_.empty()) {
    std::for_each(std::execution::par_unseq, targets_.begin(), targets_.end(),
                  draw_armor);
  }
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
  targets_.clear();
  FindArmor(frame);
  return targets_;
}

void SnipeDetector::VisualizeResult(const cv::Mat &output, int verbose) {
  VisualizeArmor(output, verbose > 2);
}
