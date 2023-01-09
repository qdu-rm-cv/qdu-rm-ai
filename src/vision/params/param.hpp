#pragma once

template <typename ParamStructInt, typename ParamStructDouble>
class Param {
 public:
  ParamStructInt parami_;
  ParamStructDouble paramd_;

  virtual ParamStructDouble TransformToDouble() = 0;

  virtual bool Read(const std::string &params_path) = 0;
  virtual void Write(const std::string &params_path) = 0;
};

// TODO(C.Meng) : PredictorParam
