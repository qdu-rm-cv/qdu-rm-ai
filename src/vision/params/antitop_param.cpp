#include "antitop_param.hpp"
AntiWhippingTopParam<double> AntiTopParam::TransformToDouble() {
  paramd_.detector_param_th = parami_.detector_param_th;
  paramd_.missing_frame_param_th = parami_.missing_frame_param_th;
  paramd_.whipping_top_param_th = parami_.whipping_top_param_th;
  paramd_.center_height_diff_low = parami_.center_height_diff_low;
  paramd_.center_height_diff_high = parami_.center_height_diff_high;
  paramd_.center_width_diff_low = parami_.center_width_diff_low;
  paramd_.center_width_diff_high = parami_.center_width_diff_high;
  return paramd_;
}

bool AntiTopParam::Read(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    parami_.detector_param_th = fs["detector_param_th"];
    parami_.missing_frame_param_th = fs["missing_frame_param_th"];
    parami_.whipping_top_param_th = fs["whipping_top_param_th"];
    parami_.center_height_diff_low = static_cast<double>
        (fs["center_height_diff_low"]);
    parami_.center_height_diff_high = static_cast<double>
        (fs["center_height_diff_high"]);
    parami_.center_width_diff_low = static_cast<double>
        (fs["center_width_diff_low"]);
    parami_.center_width_diff_high = static_cast<double>
        (fs["center_width_diff_high"]);
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

void AntiTopParam::Write(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  TransformToDouble();
  fs << "detector_param_th" << paramd_.detector_param_th;
  fs << "missing_frame_param_th" << paramd_.missing_frame_param_th;
  fs << "whipping_top_param_th" << paramd_.whipping_top_param_th;

  fs << "center_height_diff_low" << paramd_.center_height_diff_low;
  fs << "center_height_diff_high" << paramd_.center_height_diff_high;
  fs << "center_width_diff_low" << paramd_.center_width_diff_low;
  fs << "center_width_diff_high" << paramd_.center_width_diff_high;
  SPDLOG_WARN("Wrote params.");
}
