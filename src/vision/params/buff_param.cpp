#include "buff_param.hpp"

BuffDetectorParam<double> BuffParam::Transform2Double() {
  BuffDetectorParam<double> param;
  param.binary_th = param_int.binary_th;
  param.contour_size_low_th = param_int.contour_size_low_th;

  param.rect_ratio_low_th = param_int.rect_ratio_low_th / 1000.;
  param.rect_ratio_high_th = param_int.rect_ratio_high_th / 1000.;

  param.contour_center_area_low_th = param_int.contour_center_area_low_th;
  param.contour_center_area_high_th = param_int.contour_center_area_high_th;
  param.rect_center_ratio_low_th = param_int.rect_center_ratio_low_th / 1000.;
  param.rect_center_ratio_high_th = param_int.rect_center_ratio_high_th / 1000.;
  return param;
}

bool BuffParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    param_int.binary_th = fs["binary_th"];
    param_int.contour_size_low_th = static_cast<int>(fs["contour_size_low_th"]);

    param_int.rect_ratio_low_th = double(fs["rect_ratio_low_th"]) * 1000.;
    param_int.rect_ratio_high_th = double(fs["rect_ratio_high_th"]) * 1000.;

    param_int.contour_center_area_low_th = double(fs["contour_center_area_low_th"]);
    param_int.contour_center_area_high_th = double(fs["contour_center_area_high_th"]);
    param_int.rect_center_ratio_low_th = double(fs["rect_center_ratio_low_th"]) * 1000.;
    param_int.rect_center_ratio_high_th = double(fs["rect_center_ratio_high_th"]) * 1000.;
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void BuffParam::Write(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  BuffDetectorParam<double> param_double = Transform2Double();

  fs << "binary_th" << param_double.binary_th;
  fs << "contour_size_low_th"
     << static_cast<int>(param_double.contour_size_low_th);

  fs << "rect_ratio_low_th" << param_double.rect_ratio_low_th;
  fs << "rect_ratio_high_th" << param_double.rect_ratio_high_th;
  fs << "contour_center_area_low_th" << param_double.contour_center_area_low_th;
  fs << "contour_center_area_high_th" << param_double.contour_center_area_high_th;
  fs << "rect_center_ratio_low_th" << param_double.rect_center_ratio_low_th;
  fs << "rect_center_ratio_high_th" << param_double.rect_center_ratio_high_th;
  SPDLOG_WARN("Wrote params.");
}