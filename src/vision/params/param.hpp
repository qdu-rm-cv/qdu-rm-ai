#pragma once

// template <typename ParamStructInt, typename ParamStructDouble>
class Param {
 public:
  /*
  ParamStructInt param_int;
  ParamStructDouble param_double;

  virtual ParamDouble Transform2Double() = 0;
  */

  virtual bool Read(const std::string &params_path) = 0;
  virtual void Write(const std::string &params_path) const = 0;
};

// TODO : BuffDetetcorParam
// TODO : PredictorParam