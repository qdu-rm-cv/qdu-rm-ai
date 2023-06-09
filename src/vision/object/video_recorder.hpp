#pragma once
// #define thread_alone 0

#include <string>

#include "common.hpp"
#include "log.hpp"
#include "opencv2/opencv.hpp"

#ifdef thread_alone
#include <deque>
#include <thread>

#include "semaphore.hpp"
#endif

class VideoRecorder {
 private:
  cv::VideoWriter writer_;

  void Write(cv::Mat& frame) {
    if (frame.empty()) return;
    if (frame.cols != kIMAGE_WIDTH || frame.rows != kIMAGE_HEIGHT)
      cv::resize(frame, frame, cv::Size(kIMAGE_WIDTH, kIMAGE_HEIGHT));
    writer_.write(frame);
  }

#ifdef thread_alone
  component::Semaphore signal_;
  std::mutex mutex_;
  std::thread thread_;
  std::deque<cv::Mat> frame_stack_;

  void SaveLoopThread() {
    while (true) {
      if (!frame_stack_.empty()) {
        signal_.Take();
        std::lock_guard<std::mutex> lock(mutex_);
        Write(frame_stack_.front());
        frame_stack_.clear();
      }
    }
  }
#endif

 public:
  explicit VideoRecorder(int fps = 60) {
    auto path = kPATH_RUNTIME + algo::GetTimeStamp() + ".avi";

    writer_.open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
                 cv::Size(kIMAGE_WIDTH, kIMAGE_HEIGHT));

#ifdef thread_alone
    thread_ = std::thread(&VideoRecorder::SaveLoopThread, this);
#endif
  }

  ~VideoRecorder() {
#ifdef thread_alone
    thread_.join();
#endif
    writer_.release();
    SPDLOG_TRACE("Destructed.");
  }

  void Record(cv::Mat& frame) {
#ifndef thread_alone
    Write(frame);
#else
    std::lock_guard<std::mutex> lock(mutex_);
    frame_stack_.push_front(frame.clone());
    signal_.Give();
#endif
  }
};
