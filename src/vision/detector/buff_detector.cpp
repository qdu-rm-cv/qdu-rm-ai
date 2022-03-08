#include "buff_detector.hpp"

#include <cmath>
#include <execution>

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

void BuffDetector::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "binary_th" << 220;

  fs << "contour_size_low_th" << 2;
  fs << "rect_ratio_low_th" << 0.4;
  fs << "rect_ratio_high_th" << 2.5;

  fs << "hammar_rect_contour_ratio_th" << 1.2;
  fs << "hammar_rect_center_div_low_th" << 12;
  fs << "hammar_rect_center_div_high_th" << 18;

  fs << "armor_rect_center_div_low_th" << 2;
  fs << "armor_rect_center_div_high_th" << 10;
  fs << "armor_contour_rect_div_low_th" << 0.5;
  fs << "armor_contour_rect_div_high_th" << 1.6;

  fs << "contour_center_area_low_th" << 100;
  fs << "contour_center_area_high_th" << 1000;
  fs << "rect_center_ratio_low_th" << 0.6;
  fs << "rect_center_ratio_high_th" << 1.67;
  SPDLOG_DEBUG("Inited params.");
}

bool BuffDetector::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    params_.binary_th = fs["binary_th"];

    params_.contour_size_low_th = static_cast<int>(fs["contour_size_low_th"]);
    params_.rect_ratio_low_th = fs["rect_ratio_low_th"];
    params_.rect_ratio_high_th = fs["rect_ratio_high_th"];

    params_.hammar_rect_contour_ratio_th = fs["hammar_rect_contour_ratio_th"];
    params_.hammar_rect_center_div_low_th = fs["hammar_rect_center_div_low_th"];
    params_.hammar_rect_center_div_high_th =
        fs["hammar_rect_center_div_high_th"];

    params_.armor_rect_center_div_low_th = fs["armor_rect_center_div_low_th"];
    params_.armor_rect_center_div_high_th = fs["armor_rect_center_div_high_th"];
    params_.armor_contour_rect_div_low_th = fs["armor_contour_rect_div_low_th"];
    params_.armor_contour_rect_div_high_th =
        fs["armor_contour_rect_div_high_th"];

    params_.contour_center_area_low_th = fs["contour_center_area_low_th"];
    params_.contour_center_area_high_th = fs["contour_center_area_high_th"];
    params_.rect_center_ratio_low_th = fs["rect_center_ratio_low_th"];
    params_.rect_center_ratio_high_th = fs["rect_center_ratio_high_th"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void BuffDetector::MatchBuff(const cv::Mat &frame) {
  const auto start = high_resolution_clock::now();
  float center_rect_area = params_.contour_center_area_low_th * 1.5;
  tbb::concurrent_vector<Armor> armors;
  hammer_ = cv::RotatedRect();

  frame_size_ = cv::Size(frame.cols, frame.rows);

  cv::Mat channels[3], img;
  cv::split(frame, channels);

#if 1
  if (team_ == game::Team::kBLUE) {
    img = channels[0] - channels[2];
  } else if (team_ == game::Team::kRED) {
    img = channels[2] - channels[0];
  }
#else
  if (team_ == game::Team::kBLUE) {
    result = channels[0];
  } else if (team_ == game::Team::kRED) {
    result = channels[2];
  }
#endif

  cv::threshold(img, img, params_.binary_th, 255., cv::THRESH_BINARY);

  /*
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size2i(2 * params_.se_erosion + 1, 2 * params_.se_erosion + 1),
        cv::Point(params_.se_erosion, params_.se_erosion));

    cv::dilate(img, img, kernel);
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel);

  */
  cv::findContours(img, contours_, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

#if 0
  contours_poly_.resize(contours_.size());
  for (size_t i = 0; i < contours_.size(); ++i) {
    cv::approxPolyDP(cv::Mat(contours_[i]), contours_poly_[i],
                     params_.ap_erosion, true);
  }
#endif

  SPDLOG_DEBUG("Found contours: {}", contours_.size());

  auto check_armor = [&](const auto &contour) {
    if (contour.size() < static_cast<std::size_t>(params_.contour_size_low_th))
      return;

    cv::RotatedRect rect = cv::minAreaRect(contour);
    double rect_ratio = rect.size.aspectRatio();
    double contour_area = cv::contourArea(contour);
    double rect_area = rect.size.area();

    SPDLOG_DEBUG("contour_area is {}", contour_area);
    SPDLOG_DEBUG("rect_area is {}", rect_area);
    if (contour_area > params_.contour_center_area_low_th &&
        contour_area < params_.contour_center_area_high_th) {
      if (rect_ratio < params_.rect_center_ratio_high_th &&
          rect_ratio > params_.rect_center_ratio_low_th) {
        buff_.SetCenter(rect.center);
        center_rect_area = rect_area;
        SPDLOG_WARN("center's area is {}", rect_area);
        return;
      }
    }

    /* 筛选锤子 : [max(1.2 * 轮廓, 20 * R标)]  <  [锤子]  <  [80 * R标] */
    if (rect_area > params_.hammar_rect_contour_ratio_th * contour_area &&
        rect_area > params_.hammar_rect_center_div_low_th * center_rect_area &&
        rect_area < params_.hammar_rect_center_div_high_th * center_rect_area) {
      hammer_ = rect;
      SPDLOG_DEBUG("hammer_contour's area is {}", contour_area);
      return;
    }
    /* 筛选宝剑 */
    if (0 < hammer_.size.area()) {
      if (contour_area > 1.5 * hammer_.size.area()) return;
      if (rect_area > 0.7 * hammer_.size.area()) return;
    }

    SPDLOG_DEBUG("rect_ratio is {}", rect_ratio);
    if (rect_ratio < params_.rect_ratio_low_th) return;
    if (rect_ratio > params_.rect_ratio_high_th) return;

    if (rect_area < center_rect_area * params_.armor_rect_center_div_low_th)
      return;
    if (rect_area > center_rect_area * params_.armor_rect_center_div_high_th)
      return;

    if (contour_area < rect_area * params_.armor_contour_rect_div_low_th)
      return;
    if (contour_area > rect_area * params_.armor_contour_rect_div_high_th)
      return;

    SPDLOG_DEBUG("armor's area is {}", rect_area);
    Armor armor = Armor(rect);
    armor.SetModel(game::Model::kHERO);
    armors.emplace_back(armor);
  };

  std::for_each(std::execution::par_unseq, contours_.begin(), contours_.end(),
                check_armor);

  auto stop = high_resolution_clock::now();
  duration_armors_ = duration_cast<std::chrono::milliseconds>(stop - start);

  SPDLOG_DEBUG("armors.size is {}", armors.size());
  SPDLOG_DEBUG("the buff's hammer area is {}", hammer_.size.area());

  if (armors.size() > 0 && hammer_.size.area() > 0) {
    buff_.SetTarget(armors[0]);
    if (armors.size() > 1)
      for (auto armor : armors)
        if (cv::norm(hammer_.center - armor.ImageCenter()) <
            cv::norm(hammer_.center - buff_.GetTarget().ImageCenter()))
          buff_.SetTarget(armor);
    buff_.SetArmors(armors);

    targets_.emplace_back(buff_);

  } else {
    SPDLOG_WARN("can't find buff_armor");
  }

  stop = high_resolution_clock::now();
  duration_buff_ = duration_cast<std::chrono::milliseconds>(stop - start);
}

void BuffDetector::VisualizeArmors(const cv::Mat &output, bool add_lable) {
  auto target_vertices = buff_.GetTarget().ImageVertices();
  auto draw_armor = [&](Armor &armor) {
    cv::Scalar color =
        (armor.ImageVertices() == target_vertices) ? draw::kRED : draw::kGREEN;

    armor.VisualizeObject(output, add_lable, color);
  };

  tbb::concurrent_vector<Armor> armors = buff_.GetArmors();
  if (!armors.empty()) {
    std::for_each(std::execution::par_unseq, armors.begin(), armors.end(),
                  draw_armor);
  }
}

BuffDetector::BuffDetector() { SPDLOG_TRACE("Constructed."); }

BuffDetector::BuffDetector(const std::string &params_path,
                           game::Team enemy_team) {
  LoadParams(params_path);
  SetTeam(enemy_team);
  SPDLOG_TRACE("Constructed.");
}

BuffDetector::~BuffDetector() { SPDLOG_TRACE("Destructed."); }

void BuffDetector::SetTeam(game::Team enemy_team) {
  if (enemy_team == game::Team::kRED)
    team_ = game::Team::kBLUE;
  else if (enemy_team == game::Team::kBLUE)
    team_ = game::Team::kRED;
  else
    team_ = game::Team::kUNKNOWN;
}

const tbb::concurrent_vector<Buff> &BuffDetector::Detect(const cv::Mat &frame) {
  targets_.clear();
  buff_ = Buff();
  SPDLOG_DEBUG("Detecting");
  MatchBuff(frame);
  SPDLOG_DEBUG("Detected.");
  targets_.emplace_back(buff_);
  return targets_;
}

void BuffDetector::VisualizeResult(const cv::Mat &output, int verbose) {
  SPDLOG_DEBUG("Visualizeing Result.");
  if (verbose > 10) {
    cv::drawContours(output, contours_, -1, draw::kRED);
    cv::drawContours(output, contours_poly_, -1, draw::kYELLOW);
  }

  if (verbose > 1) {
    std::string label =
        cv::format("%ld armors in %ld ms.", buff_.GetArmors().size(),
                   duration_armors_.count());
    draw::VisualizeLabel(output, label, 1);

    label = cv::format("Match buff in %ld ms.", duration_buff_.count());
    draw::VisualizeLabel(output, label, 2);
  }
  if (verbose > 2) {
    cv::Point2f vertices[4];
    hammer_.points(vertices);
    for (std::size_t i = 0; i < 4; ++i)
      cv::line(output, vertices[i], vertices[(i + 1) % 4], draw::kYELLOW);

    cv::drawMarker(output, buff_.GetCenter(), draw::kYELLOW,
                   cv::MARKER_DIAMOND);
  }
  VisualizeArmors(output, verbose > 2);
  SPDLOG_DEBUG("Visualized.");
}