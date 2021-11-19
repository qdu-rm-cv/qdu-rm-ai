#include "armor_param.hpp"

ArmorDetectorParam<double> ArmorParam::transform2Double() {
  ArmorDetectorParam<double> param;
  param.binary_th = param_int.binary_th;
  param.contour_size_low_th = param_int.contour_size_low_th;

  param.contour_area_low_th = param_int.contour_area_low_th / 1000.;
  param.contour_area_high_th = param_int.contour_area_high_th / 1000.;
  param.bar_area_low_th = param_int.bar_area_low_th / 1000.;
  param.bar_area_high_th = param_int.bar_area_high_th / 1000.;
  param.angle_high_th = param_int.angle_high_th / 1000.;
  param.aspect_ratio_low_th = param_int.aspect_ratio_low_th / 1000.;
  param.aspect_ratio_high_th = param_int.aspect_ratio_high_th / 1000.;

  param.angle_diff_th = param_int.angle_diff_th / 1000.;
  param.length_diff_th = param_int.length_diff_th / 1000.;
  param.height_diff_th = param_int.height_diff_th / 1000.;
  param.area_diff_th = param_int.area_diff_th / 1000.;
  param.center_dist_low_th = param_int.center_dist_low_th / 1000.;
  param.center_dist_high_th = param_int.center_dist_high_th / 1000.;
  return param;
}

bool ArmorParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    param_int.binary_th = fs["binary_th"];
    param_int.contour_size_low_th = fs["contour_size_low_th"];

    param_int.contour_area_low_th = double(fs["contour_area_low_th"]) * 1000.;
    param_int.contour_area_high_th = double(fs["contour_area_high_th"]) * 1000.;
    param_int.bar_area_low_th = double(fs["bar_area_low_th"]) * 1000.;
    param_int.bar_area_high_th = double(fs["bar_area_high_th"]) * 1000.;
    param_int.angle_high_th = double(fs["angle_high_th"]) * 1000.;
    param_int.aspect_ratio_low_th = double(fs["aspect_ratio_low_th"]) * 1000.;
    param_int.aspect_ratio_high_th = double(fs["aspect_ratio_high_th"]) * 1000.;

    param_int.angle_diff_th = double(fs["angle_diff_th"]) * 1000.;
    param_int.length_diff_th = double(fs["length_diff_th"]) * 1000.;
    param_int.height_diff_th = double(fs["height_diff_th"]) * 1000.;
    param_int.area_diff_th = double(fs["area_diff_th"]) * 1000.;
    param_int.center_dist_low_th = double(fs["center_dist_low_th"]) * 1000.;
    param_int.center_dist_high_th = double(fs["center_dist_high_th"]) * 1000.;
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void ArmorParam::Write(const std::string &params_path) const {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "binary_th" << param_int.binary_th;
  fs << "contour_size_low_th"
     << static_cast<int>(param_int.contour_size_low_th);

  fs << "contour_area_low_th" << param_int.contour_area_low_th / 1000.;
  fs << "contour_area_high_th" << param_int.contour_area_high_th / 1000.;
  fs << "bar_area_low_th" << param_int.bar_area_low_th / 1000.;
  fs << "bar_area_high_th" << param_int.bar_area_high_th / 1000.;
  fs << "angle_high_th" << param_int.angle_high_th / 1000.;
  fs << "aspect_ratio_low_th" << param_int.aspect_ratio_low_th / 1000.;
  fs << "aspect_ratio_high_th" << param_int.aspect_ratio_high_th / 1000.;

  fs << "angle_diff_th" << param_int.angle_diff_th / 1000.;
  fs << "length_diff_th" << param_int.length_diff_th / 1000.;
  fs << "height_diff_th" << param_int.height_diff_th / 1000.;
  fs << "area_diff_th" << param_int.area_diff_th / 1000.;
  fs << "center_dist_low_th" << param_int.center_dist_low_th / 1000.;
  fs << "center_dist_high_th" << param_int.center_dist_high_th / 1000.;
  SPDLOG_WARN("Wrote params.");
}