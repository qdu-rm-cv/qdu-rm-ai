#pragma once
#include <string>

#include "opencv2/core/persistence.hpp"
#include "param.hpp"
#include "spdlog/spdlog.h"

template <typename Type>
struct BuffDetectorParam {
  Type binary_th;

  int contour_size_low_th;
  Type rect_ratio_low_th;
  Type rect_ratio_high_th;

  Type hammar_rect_contour_ratio_th;
  Type hammar_rect_center_div_low_th;
  Type hammar_rect_center_div_high_th;

  Type armor_rect_center_div_low_th;
  Type armor_rect_center_div_high_th;
  Type armor_contour_rect_div_low_th;
  Type armor_contour_rect_div_high_th;

  Type contour_center_area_low_th;
  Type contour_center_area_high_th;
  Type rect_center_ratio_low_th;
  Type rect_center_ratio_high_th;
};

class BuffParam
    : public Param<BuffDetectorParam<int>, BuffDetectorParam<double>> {
 public:
  BuffDetectorParam<double> TransformToDouble();

  bool Read(const std::string &params_path) override;
  void Write(const std::string &params_path) override;
};