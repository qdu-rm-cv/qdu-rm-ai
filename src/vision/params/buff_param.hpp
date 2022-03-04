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