#pragma once
#include <string>

#include "param.hpp"
#include "spdlog/spdlog.h"

template <typename Type>
struct ArmorDetectorParam {
  int binary_th;
  int contour_size_low_th;
  Type contour_area_low_th;
  Type contour_area_high_th;
  Type bar_area_low_th;
  Type bar_area_high_th;
  Type angle_high_th;
  Type aspect_ratio_low_th;
  Type aspect_ratio_high_th;
  Type angle_diff_th;
  Type length_diff_th;
  Type height_diff_th;
  Type area_diff_th;
  Type center_dist_low_th;
  Type center_dist_high_th;
};

class ArmorParam
    : public Param<ArmorDetectorParam<int>, ArmorDetectorParam<double>> {
 public:
  ArmorDetectorParam<double> TransformToDouble();

  bool Read(const std::string &params_path) override;
  void Write(const std::string &params_path) override;
};
