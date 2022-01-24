#include "orecube_detector.hpp"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

namespace {

const auto kCV_FONT = cv::FONT_HERSHEY_SIMPLEX;
const cv::Scalar kGREEN(0., 255., 0.);
const cv::Scalar kRED(0., 0., 255.);
const cv::Scalar kYELLOW(0., 255., 255.);

}  // namespace

void OreCubeDetector::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "a" << 0;
  fs << "b" << 0;
  SPDLOG_DEBUG("Inited params.");
}

bool OreCubeDetector::PrepareParams(const std::string &params_path) {
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

void OreCubeDetector::FindOreCube(const cv::Mat &frame) {
  (void)frame;
  targets_.emplace_back(OreCube());
}

void OreCubeDetector::VisualizeOreCube(const cv::Mat &output, bool add_lable) {
  (void)output;
  (void)add_lable;
}

OreCubeDetector::OreCubeDetector() { SPDLOG_TRACE("Constructed."); }

OreCubeDetector::OreCubeDetector(const std::string &params_path) {
  LoadParams(params_path);
  SPDLOG_TRACE("Constructed.");
}

OreCubeDetector::~OreCubeDetector() { SPDLOG_TRACE("Destructed."); }

const tbb::concurrent_vector<OreCube> &OreCubeDetector::Detect(
    const cv::Mat &frame) {
  targets_.clear();
  FindOreCube(frame);
  return targets_;
}

void OreCubeDetector::VisualizeResult(const cv::Mat &output, int verbose) {
  VisualizeOreCube(output, verbose > 2);
}
