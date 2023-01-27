#include "buff_param.hpp"

#include "opencv2/core/persistence.hpp"

BuffDetectorParam<double> BuffParam::TransformToDouble() {
  paramd_.binary_th = parami_.binary_th;
  paramd_.contour_size_low_th = parami_.contour_size_low_th;

  paramd_.rect_ratio_low_th = parami_.rect_ratio_low_th / 1000.;
  paramd_.rect_ratio_high_th = parami_.rect_ratio_high_th / 1000.;

  paramd_.contour_center_area_low_th = parami_.contour_center_area_low_th;
  paramd_.contour_center_area_high_th = parami_.contour_center_area_high_th;
  paramd_.rect_center_ratio_low_th = parami_.rect_center_ratio_low_th / 1000.;
  paramd_.rect_center_ratio_high_th = parami_.rect_center_ratio_high_th / 1000.;
  return paramd_;
}

bool BuffParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    parami_.binary_th = fs["binary_th"];
    parami_.contour_size_low_th = static_cast<int>(fs["contour_size_low_th"]);

    parami_.rect_ratio_low_th =
        static_cast<double>(fs["rect_ratio_low_th"]) * 1000.;
    parami_.rect_ratio_high_th =
        static_cast<double>(fs["rect_ratio_high_th"]) * 1000.;

    parami_.contour_center_area_low_th =
        static_cast<double>(fs["contour_center_area_low_th"]);
    parami_.contour_center_area_high_th =
        static_cast<double>(fs["contour_center_area_high_th"]);
    parami_.rect_center_ratio_low_th =
        static_cast<double>(fs["rect_center_ratio_low_th"]) * 1000.;
    parami_.rect_center_ratio_high_th =
        static_cast<double>(fs["rect_center_ratio_high_th"]) * 1000.;
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void BuffParam::Write(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  TransformToDouble();

  fs << "binary_th" << paramd_.binary_th;
  fs << "contour_size_low_th" << static_cast<int>(paramd_.contour_size_low_th);

  fs << "rect_ratio_low_th" << paramd_.rect_ratio_low_th;
  fs << "rect_ratio_high_th" << paramd_.rect_ratio_high_th;
  fs << "contour_center_area_low_th" << paramd_.contour_center_area_low_th;
  fs << "contour_center_area_high_th" << paramd_.contour_center_area_high_th;
  fs << "rect_center_ratio_low_th" << paramd_.rect_center_ratio_low_th;
  fs << "rect_center_ratio_high_th" << paramd_.rect_center_ratio_high_th;
  SPDLOG_WARN("Wrote params.");
}
