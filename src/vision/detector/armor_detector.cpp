#include "armor_detector.hpp"

#include <execution>

#include "spdlog/spdlog.h"

void ArmorDetector::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "binary_th" << 220;
  // fs << "se_erosion" << 5;
  // fs << "ap_erosion" << 1.;

  fs << "contour_size_low_th" << 0;
  fs << "contour_area_low_th" << 0.00001;
  fs << "contour_area_high_th" << 0.001;
  fs << "bar_area_low_th" << 0.00001;
  fs << "bar_area_high_th" << 0.001;
  fs << "angle_high_th" << 60;
  fs << "aspect_ratio_low_th" << 2;
  fs << "aspect_ratio_high_th" << 1000;

  fs << "angle_diff_th" << 0.2;
  fs << "length_diff_th" << 0.2;
  fs << "height_diff_th" << 0.2;
  fs << "center_y_dist" << 0.2;
  fs << "area_diff_th" << 0.6;
  fs << "center_dist_low_th" << 1;
  fs << "center_dist_high_th" << 4;
  SPDLOG_DEBUG("Inited params.");
}

bool ArmorDetector::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    params_.binary_th = fs["binary_th"];
    // params_.se_erosion = fs["se_erosion"];
    // params_.ap_erosion = fs["ap_erosion"];

    params_.contour_size_low_th = static_cast<int>(fs["contour_size_low_th"]);
    params_.contour_area_low_th = fs["contour_area_low_th"];
    params_.contour_area_high_th = fs["contour_area_high_th"];
    params_.bar_area_low_th = fs["bar_area_low_th"];
    params_.bar_area_high_th = fs["bar_area_high_th"];
    params_.angle_high_th = fs["angle_high_th"];
    params_.aspect_ratio_low_th = fs["aspect_ratio_low_th"];
    params_.aspect_ratio_high_th = fs["aspect_ratio_high_th"];

    params_.angle_diff_th = fs["angle_diff_th"];
    params_.length_diff_th = fs["length_diff_th"];
    params_.height_diff_th = fs["height_diff_th"];
    params_.center_y_dist = fs["center_y_dist"];
    params_.area_diff_th = fs["area_diff_th"];
    params_.center_dist_low_th = fs["center_dist_low_th"];
    params_.center_dist_high_th = fs["center_dist_high_th"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void ArmorDetector::FindLightBars(const cv::Mat &frame) {
  duration_bars_.Start();
  lightbars_.clear();
  targets_.clear();

  frame_size_ = frame.size();
  const double frame_area = frame_size_.area();
  cv::Mat result;
  frame.copyTo(result);
  /*
  std::vector<cv::Mat> channels(3);
  cv::Mat result;
  cv::split(frame, channels);

  if (enemy_team_ == game::Team::kUNKNOWN) {
    SPDLOG_ERROR("enemy_team_ is {}", game::ToString(enemy_team_));
    return;
  }

#if 0
  if (enemy_team_ == game::Team::kBLUE) {
    result = channels[0];
  } else if (enemy_team_ == game::Team::kRED) {
    result = channels[2];
  }
#else
  if (enemy_team_ == game::Team::kBLUE) {
    result = channels[0] - channels[2];
  } else if (enemy_team_ == game::Team::kRED) {
    result = channels[2] - channels[0];
  }
#endif*/
  cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
  cv::threshold(result, result, params_.binary_th, 255., cv::THRESH_BINARY);
  /*
    if (params_.se_erosion >= 0.) {
      cv::Mat kernel = cv::getStructuringElement(
          cv::MORPH_ELLIPSE,
          cv::Size(2 * params_.se_erosion + 1, 2 * params_.se_erosion + 1));
      cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
    }
  */
  cv::findContours(result, contours_, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_TC89_KCOS);

#if 0 /* 平滑轮廓应该有用，但是这里简化轮廓没用 */
  contours_poly_.resize(contours_.size());
  for (size_t k = 0; k < contours_.size(); ++k) {
    cv::approxPolyDP(cv::Mat(contours_[k]), contours_poly_[k],
                     params_.ap_erosion, true);
  }
#endif

  SPDLOG_DEBUG("Found contours: {}", contours_.size());

  /* 检查轮廓是否为灯条 */
  auto check_lightbar = [&](const auto &contour) {
    /* 通过轮廓大小先排除明显不是的 */
    if (contour.size() < static_cast<std::size_t>(params_.contour_size_low_th))
      return;

    /* 只留下轮廓大小在一定比例内的 */
    const double c_area = cv::contourArea(contour) / frame_area;
    SPDLOG_INFO("c_area is {}", c_area);
    if (c_area < params_.contour_area_low_th) return;
    if (c_area > params_.contour_area_high_th) return;

    LightBar potential_bar(cv::minAreaRect(contour));

    /* 灯条倾斜角度不能太大 */
    SPDLOG_INFO("angle is {}", std::abs(potential_bar.ImageAngle()));
    if (std::abs(potential_bar.ImageAngle()) > params_.angle_high_th) return;

    /* 灯条在画面中的大小要满足条件 */
    const double bar_area = potential_bar.Area() / frame_area;
    SPDLOG_INFO("bar_area is {}", bar_area);
    if (bar_area < params_.bar_area_low_th) return;
    if (bar_area > params_.bar_area_high_th) return;

    /* 灯条的长宽比要满足条件 */
    const double aspect_ratio = potential_bar.ImageAspectRatio();
    SPDLOG_INFO("aspect_ratio is {}", aspect_ratio);
    if (aspect_ratio < params_.aspect_ratio_low_th) return;
    if (aspect_ratio > params_.aspect_ratio_high_th) return;
    /*进行灯条的颜色验证*/
    auto rect = potential_bar.GetLightBarROI();
    if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= frame.cols &&
        0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= frame.rows) {
      auto ROI = frame(rect);
      int blue_sum = 0;
      int red_sum = 0;
      for (int i = 0; i < ROI.cols; i++) {
        for (int j = 0; j < ROI.rows; j++) {
          // if (cv::pointPolygonTest(contour, cv::Point2f(i, j), false) >= 0) {
          // if point is inside contour
          blue_sum += ROI.at<cv::Vec3b>(i, j)[0];
          red_sum += ROI.at<cv::Vec3b>(i, j)[2];
          //}
        }
      }
      bool color = blue_sum > red_sum ? true : false;
      if (enemy_team_ == game::Team::kBLUE && color) return;
      if (enemy_team_ == game::Team::kRED && !color) return;
    } else {
      return;
    }
    lightbars_.emplace_back(potential_bar);
  };
  /* 并行验证灯条 */
  std::for_each(std::execution::par_unseq, contours_.begin(), contours_.end(),
                check_lightbar);

  /* 从左到右排列找到的灯条 */
  std::sort(lightbars_.begin(), lightbars_.end(),
            [](LightBar &bar1, LightBar &bar2) {
              return bar1.ImageCenter().x < bar2.ImageCenter().x;
            });

  /* 记录运行时间 */
  duration_bars_.Calc("Find Bars");
}

void ArmorDetector::MatchLightBars(const cv::Mat frame) {
  duration_armors_.Start();
  for (auto iti = lightbars_.begin(); iti != lightbars_.end(); ++iti) {
    for (auto itj = iti + 1; itj != lightbars_.end(); ++itj) {
      /* 两灯条角度差异 */

      const double angle_diff =
          algo::RelativeDifference(iti->ImageAngle(), itj->ImageAngle());

      /* 灯条是否朝同一侧倾斜 */
      const bool same_side = (iti->ImageAngle() * itj->ImageAngle()) > 0;

      if (same_side) {
        if (angle_diff > params_.angle_diff_th) continue;
      } else {
        /* 两侧时限制更严格 */
        if (angle_diff > (params_.angle_diff_th / 2.)) continue;
      }

      /* 灯条长度差异 */
      const double length_diff =
          algo::RelativeDifference(iti->Length(), itj->Length());
      SPDLOG_INFO("length_diff is {}", length_diff);
      if (length_diff > params_.length_diff_th) continue;

      /* 灯条高度差异 */
      const double height_diff =
          algo::RelativeDifference(iti->ImageCenter().y, itj->ImageCenter().y);
      SPDLOG_INFO("height_diff is {}", height_diff);
      if (height_diff > (params_.height_diff_th * frame_size_.height)) continue;

      /* 灯条面积差异 */
      const double area_diff =
          algo::RelativeDifference(iti->Area(), itj->Area());
      if (area_diff > params_.area_diff_th) continue;

      /* 中心高度差 */
      const double length = (iti->Length() + itj->Length()) / 2;
      const double center_y_dis =
          abs(iti->image_center_.y - itj->image_center_.y);
      SPDLOG_INFO("center_y_dis is {}", center_y_dis);
      if (center_y_dis >= length * params_.center_y_dist) continue;

      /* 灯条中心距离 */
      const double center_dist =
          cv::norm(iti->ImageCenter() - itj->ImageCenter());
      const double l = (iti->Length() + itj->Length()) / 2.;
      if (center_dist < l * params_.center_dist_low_th) continue;
      if (center_dist > l * params_.center_dist_high_th) continue;
      auto armor = Armor(*iti, *itj);
      // cv::imshow("Armor Face", armor.Face(frame));
      // cv::Mat face = armor.Face(frame);
      // int id = matcher.GetArmorId(face);
      // cv::waitKey(1);
      // if (id == 0) continue;
      // // armor.SetModel(game::Model::kINFANTRY);
      targets_.emplace_back(armor);
      break;
    }
  }

  duration_armors_.Calc("Find Armors");
}

ArmorDetector::ArmorDetector() { SPDLOG_TRACE("Constructed."); }

ArmorDetector::ArmorDetector(const std::string &params_path,
                             game::Team enemy_team) {
  LoadParams(params_path);
  SetEnemyTeam(enemy_team);
  SPDLOG_TRACE("Constructed.");
}

ArmorDetector::~ArmorDetector() { SPDLOG_TRACE("Destructed."); }

void ArmorDetector::SetEnemyTeam(game::Team enemy_team) {
  enemy_team_ = enemy_team;
}

const tbb::concurrent_vector<Armor> &ArmorDetector::Detect(
    const cv::Mat &frame) {
  SPDLOG_DEBUG("Detecting");
  FindLightBars(frame);
  MatchLightBars(frame);
  SPDLOG_DEBUG("Detected.");
  return targets_;
}

void ArmorDetector::VisualizeResult(const cv::Mat &output, int verbose) {
  auto draw_lightbar = [&](LightBar &bar) {
    bar.VisualizeObject(output, verbose > 2, draw::kGREEN, cv::MARKER_CROSS);
  };
  auto draw_armor = [&](Armor &armor) {
    armor.VisualizeObject(output, verbose > 2);
  };

  if (verbose > 0) {
    cv::drawContours(output, contours_, -1, draw::kRED);
    cv::drawContours(output, contours_poly_, -1, draw::kYELLOW);
  }
  if (verbose > 1) {
    std::string label = cv::format("%ld bars in %ld ms.", lightbars_.size(),
                                   duration_bars_.Count());
    draw::VisualizeLabel(output, label, 1);

    label = cv::format("%ld armors in %ld ms.", targets_.size(),
                       duration_armors_.Count());
    draw::VisualizeLabel(output, label, 2);
  }

  if (!lightbars_.empty()) {
    std::for_each(std::execution::par_unseq, lightbars_.begin(),
                  lightbars_.end(), draw_lightbar);
  }
  if (!targets_.empty()) {
    std::for_each(std::execution::par_unseq, targets_.begin(), targets_.end(),
                  draw_armor);
  }
}
