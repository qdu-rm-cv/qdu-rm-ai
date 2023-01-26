#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "semaphore.hpp"
#include "spdlog/spdlog.h"
#include "timer.hpp"

class Camera {
 private:
  component::Recorder recorder_ = component::Recorder("CameraThread");
  virtual void GrabPrepare() = 0;
  virtual void GrabLoop() = 0;

  void GrabThread() {
    SPDLOG_DEBUG("[GrabThread] Started.");
    frame_signal_.Init();
    GrabPrepare();
    while (grabing) {
      GrabLoop();
      recorder_.Record();
    }
    SPDLOG_DEBUG("[GrabThread] Stoped.");
  }

  virtual bool OpenPrepare(unsigned int index) = 0;

 public:
  unsigned int frame_h_, frame_w_;
  component::Semaphore frame_signal_;

  bool grabing = false;
  std::thread grab_thread_;
  std::mutex frame_stack_mutex_;
  std::deque<cv::Mat> frame_stack_;

  /**
   * @brief 设置相机参数
   *
   * @param width 输出图像宽度
   * @param height 输出图像高度
   */
  void Setup(unsigned int width, unsigned int height) {
    frame_w_ = width;
    frame_h_ = height;

    // TODO(Somebody) : 配置相机输入输出
  }

  /**
   * @brief 打开相机设备
   *
   * @param index 相机索引号
   * @return true 打开成功
   * @return false 打开失败
   */
  bool Open(unsigned int index) {
    if (OpenPrepare(index)) {
      grabing = true;
      grab_thread_ = std::thread(&Camera::GrabThread, this);
      return true;
    }
    return false;
  }

  /**
   * @brief Get the Frame object
   *
   * @return cv::Mat 拍摄的图像
   */
  virtual bool GetFrame(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(frame_stack_mutex_);
    if (!frame_stack_.empty()) {
      cv::resize(frame_stack_.front(), frame, cv::Size(frame_w_, frame_h_));
      frame_stack_.clear();
    } else {
      // SPDLOG_ERROR("Empty frame stack!");
      return false;
    }
    return true;
  }

  /**
   * @brief 关闭相机设备
   *
   * @return int 状态代码
   */
  virtual int Close() = 0;
};
