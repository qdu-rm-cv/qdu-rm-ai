#pragma once

#include <NvInfer.h>
#include <NvInferRuntimeCommon.h>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace TRT {

class TRTDeleter {
 public:
  template <typename T>
  void operator()(T *obj) const;
};

class TRTLogger : public nvinfer1::ILogger {
 public:
  TRTLogger() = default;
  ~TRTLogger() = default;

  void log(Severity severity, const char *msg) noexcept override;
  int GetVerbosity();
};

struct Detection {
  float x_ctr;
  float y_ctr;
  float w;
  float h;
  float conf;  // bbox_conf * cls_conf
  float class_id;
};

}  // namespace TRT

class TrtDetector {
  template <typename T>
  using UniquePtr = std::unique_ptr<T, TRT::TRTDeleter>;

 private:
  std::string onnx_file_path_;
  std::string engine_path_;

  TRT::TRTLogger logger_;

  UniquePtr<nvinfer1::ICudaEngine> engine_;
  UniquePtr<nvinfer1::IExecutionContext> context_;

  float conf_thresh_, nms_thresh_;

  std::vector<void *> bindings_;
  std::vector<size_t> bingings_size_;
  int idx_in_, idx_out_;
  nvinfer1::Dims dim_in_, dim_out_;
  int nc;

  std::vector<TRT::Detection> PostProcess(std::vector<float> prob);

  bool CreateEngine();
  bool LoadEngine();
  bool SaveEngine();
  bool CreateContex();
  bool InitMemory();

 public:
  TrtDetector();
  TrtDetector(const std::string &onnx_file_path, float conf_thresh = 0.5f,
              float nms_thresh = 0.5f);
  ~TrtDetector();

  void SetOnnxPath(const std::string &onnx_file_path);
  void Init(float conf_thresh = 0.5f, float nms_thresh = 0.5f);

  std::vector<TRT::Detection> Infer(const cv::Mat &);

  bool TestInfer();
};
