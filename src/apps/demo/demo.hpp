#pragma once

#include "app/app.hpp"
#include "opencv2/videoio.hpp"
#include "spdlog/spdlog.h"

class Demo : private App {
 private:
  cv::VideoCapture cam_;
  cv::VideoWriter writer_;

  bool Prepare(const std::string& video_path, const std::string& writer_path) {
    cam_.open(video_path);
    cam_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cam_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (cam_.isOpened()) {
      SPDLOG_WARN("Camera is opened");
      return true;
    } else {
      SPDLOG_ERROR("Cam has wrong address : {}", video_path);
      return false;
    }

    const int codec = cam_.get(cv::CAP_PROP_FOURCC);
    double fps = cam_.get(cv::CAP_PROP_FPS);
    cv::Size f_size(cam_.get(cv::CAP_PROP_FRAME_WIDTH),
                    cam_.get(cv::CAP_PROP_FRAME_HEIGHT));

    writer_.open(writer_path, codec, fps, f_size);
    if (!writer_.isOpened()) {
      SPDLOG_ERROR("Writer has wrong address : {}", writer_path);
      return false;
    }
    return true;
  }

 public:
  explicit Demo(const std::string& log_path) : App(log_path) {
    SPDLOG_TRACE("Constructed Demo.");
  }
  Demo(const std::string& log_path, const std::string& video_path,
       const std::string& writer_path)
      : App(log_path) {
    Open(video_path, writer_path);
    SPDLOG_TRACE("Constructed Demo.");
  }
  ~Demo() {
    cam_.release();
    writer_.release();
    SPDLOG_TRACE("Destructed Demo.");
  }

  void Open(const std::string& video_path, const std::string& writer_path) {
    if (Prepare(video_path, writer_path)) {
      SPDLOG_DEBUG("open device success.");
    } else {
      SPDLOG_WARN("False opened.");
    }
  }

  const cv::Mat Read() {
    cv::Mat frame;
    cam_ >> frame;
    return frame;
  }
  void Write(cv::Mat frame) { writer_.write(frame); }
  virtual void Run() = 0;
};
