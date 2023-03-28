#pragma once
#include <string>

#include "opencv2/opencv.hpp"
#include "param.hpp"
#include "spdlog/spdlog.h"

template <typename Type>
struct AntiWhippingTopParam {
  Type center_height_diff_low;
  Type center_height_diff_high;
  Type center_width_diff_low;
  Type center_width_diff_high;
  int detector_param_th;
  int whipping_top_param_th;
  int missing_frame_param_th;
};

class AntiTopParam
    : public Param<AntiWhippingTopParam<int>, AntiWhippingTopParam<double>> {
 public:
  AntiWhippingTopParam<double> TransformToDouble();

  bool Read(const std::string &params_path) override;
  void Write(const std::string &params_path) override;
};
