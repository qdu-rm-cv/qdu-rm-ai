#include "armor_predictor.hpp"

void ArmorPredictor::MatchArmor() {
  cv::Point2d center = armor_.ImageCenter();
  cv::Size2d size = armor_.GetRect().size;
  cv::Point2d predict_pt = filter_.Predict(center);
  cv::RotatedRect rect(predict_pt, size, armor_.GetRect().angle);
  predicts_.emplace_back(Armor(rect));
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

void ArmorPredictor::VisualizePrediction(const cv::Mat &output, int add_lable) {
  for (auto predict : predicts_) {
    SPDLOG_DEBUG("{}, {}", predict.ImageCenter().x, predict.ImageCenter().y);

    if (add_lable > 0) {
      auto vertices = predict.ImageVertices();
      for (std::size_t i = 0; i < vertices.size(); ++i)
        cv::line(output, vertices[i], vertices[(i + 1) % 4], kYELLOW, 8);
      std::ostringstream buf;
      buf << predict.ImageCenter().x << ", " << predict.ImageCenter().y;
      cv::putText(output, buf.str(), vertices[1], kCV_FONT, 1.0, kRED);
    }
    if (add_lable > 2) {
      std::string label;
      int baseLine, v_pos = 0;

      label = cv::format("Direction %s in %ld ms.",
                         component::DirectionToString(direction_).c_str(),
                         duration_direction_.count());
      cv::Size text_size = cv::getTextSize(label, kCV_FONT, 1.0, 2, &baseLine);
      v_pos += 3 * static_cast<int>(1.3 * text_size.height);
      cv::putText(output, label, cv::Point(0, v_pos), kCV_FONT, 1.0, kGREEN);

      label = cv::format("Find predict in %ld ms.", duration_predict_.count());
      v_pos += static_cast<int>(1.3 * text_size.height);
      cv::putText(output, label, cv::Point(0, v_pos), kCV_FONT, 1.0, kGREEN);
    }
  }
}
