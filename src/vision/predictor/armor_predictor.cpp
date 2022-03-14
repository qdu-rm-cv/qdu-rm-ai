#include "armor_predictor.hpp"

#include <execution>

#include "opencv2/gapi/render.hpp"

void ArmorPredictor::MatchArmor() {
  duration_predict_.Start();

  cv::Point2d center = armor_.ImageCenter();
  cv::Size2d size = armor_.GetRect().size;
  cv::Point2d predict_pt = filter_.Predict(center);
  cv::RotatedRect rect(predict_pt, size, armor_.GetRect().angle);

  Armor armor(rect);
  armor.SetModel(armor_.GetModel());
  predicts_.emplace_back(armor);

  duration_predict_.Calc("Predict Armor");
}

void ArmorPredictor::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "a" << 0;
  fs << "b" << 0;
  SPDLOG_DEBUG("Inited params.");
}

bool ArmorPredictor::PrepareParams(const std::string &params_path) {
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

ArmorPredictor::ArmorPredictor() { SPDLOG_TRACE("Constructed."); }

ArmorPredictor::ArmorPredictor(const std::string &param) {
  LoadParams(param);
  SPDLOG_TRACE("Constructed.");
}

ArmorPredictor::~ArmorPredictor() { SPDLOG_TRACE("Destructed."); }

void ArmorPredictor::SetArmor(const Armor &armor) { armor_ = armor; }

void ArmorPredictor::SetArmors(const tbb::concurrent_vector<Armor> &armors) {
  armors_ = armors;
}

const tbb::concurrent_vector<Armor> &ArmorPredictor::Predict() {
  predicts_.clear();
  MatchArmor();
  return predicts_;
}

void ArmorPredictor::VisualizePrediction(const cv::Mat &output, int verbose) {
  auto draw_armor = [&](Armor &armor) {
    auto prims = armor.VisualizeObject(verbose > 0);
    for (auto &prim : prims) prims_.emplace_back(prim);
  };

  if (!predicts_.empty()) {
    std::for_each(std::execution::par_unseq, predicts_.begin(), predicts_.end(),
                  draw_armor);
  }
  if (verbose > 1) {
    std::string label =
        cv::format("Find predict in %ld ms.", duration_predict_.Count());
    prims_.emplace_back(draw::VisualizeLabel(label, 3));
  }
  cv::Mat frame = output.clone();
  cv::gapi::wip::draw::render(frame, prims_);
}
