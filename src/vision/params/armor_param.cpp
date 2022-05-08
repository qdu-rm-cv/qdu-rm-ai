#include "armor_param.hpp"

ArmorDetectorParam<double> ArmorParam::TransformToDouble() {
  paramd_.binary_th = parami_.binary_th;
  paramd_.contour_size_low_th = parami_.contour_size_low_th;

  paramd_.contour_area_low_th = parami_.contour_area_low_th / 100000.;
  paramd_.contour_area_high_th = parami_.contour_area_high_th / 100000.;
  paramd_.bar_area_low_th = parami_.bar_area_low_th / 100000.;
  paramd_.bar_area_high_th = parami_.bar_area_high_th / 100000.;
  paramd_.angle_high_th = parami_.angle_high_th / 10.;
  paramd_.aspect_ratio_low_th = parami_.aspect_ratio_low_th / 10.;
  paramd_.aspect_ratio_high_th = parami_.aspect_ratio_high_th / 10.;

  paramd_.angle_diff_th = parami_.angle_diff_th / 1000.;
  paramd_.length_diff_th = parami_.length_diff_th / 1000.;
  paramd_.height_diff_th = parami_.height_diff_th / 100000.;
  paramd_.area_diff_th = parami_.area_diff_th / 1000.;
  paramd_.center_dist_low_th = parami_.center_dist_low_th / 1000.;
  paramd_.center_dist_high_th = parami_.center_dist_high_th / 1000.;
  return paramd_;
}

bool ArmorParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    parami_.binary_th = fs["binary_th"];
    parami_.contour_size_low_th = fs["contour_size_low_th"];

    parami_.contour_area_low_th = double(fs["contour_area_low_th"]) * 100000.;
    parami_.contour_area_high_th = double(fs["contour_area_high_th"]) * 100000.;
    parami_.bar_area_low_th = double(fs["bar_area_low_th"]) * 100000.;
    parami_.bar_area_high_th = double(fs["bar_area_high_th"]) * 100000.;
    parami_.angle_high_th = double(fs["angle_high_th"]) * 10.;
    parami_.aspect_ratio_low_th = double(fs["aspect_ratio_low_th"]) * 10.;
    parami_.aspect_ratio_high_th = double(fs["aspect_ratio_high_th"]) * 10.;

    parami_.angle_diff_th = double(fs["angle_diff_th"]) * 1000.;
    parami_.length_diff_th = double(fs["length_diff_th"]) * 1000.;
    parami_.height_diff_th = double(fs["height_diff_th"]) * 100000.;
    parami_.area_diff_th = double(fs["area_diff_th"]) * 1000.;
    parami_.center_dist_low_th = double(fs["center_dist_low_th"]) * 1000.;
    parami_.center_dist_high_th = double(fs["center_dist_high_th"]) * 1000.;
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void ArmorParam::Write(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  TransformToDouble();
  fs << "binary_th" << paramd_.binary_th;
  fs << "contour_size_low_th" << static_cast<int>(paramd_.contour_size_low_th);

  fs << "contour_area_low_th" << paramd_.contour_area_low_th;
  fs << "contour_area_high_th" << paramd_.contour_area_high_th;
  fs << "bar_area_low_th" << paramd_.bar_area_low_th;
  fs << "bar_area_high_th" << paramd_.bar_area_high_th;
  fs << "angle_high_th" << paramd_.angle_high_th;
  fs << "aspect_ratio_low_th" << paramd_.aspect_ratio_low_th;
  fs << "aspect_ratio_high_th" << paramd_.aspect_ratio_high_th;
  fs << "angle_diff_th" << paramd_.angle_diff_th;
  fs << "length_diff_th" << paramd_.length_diff_th;
  fs << "height_diff_th" << paramd_.height_diff_th;
  fs << "area_diff_th" << paramd_.area_diff_th;
  fs << "center_dist_low_th" << paramd_.center_dist_low_th;
  fs << "center_dist_high_th" << paramd_.center_dist_high_th;
  SPDLOG_WARN("Wrote params.");
}