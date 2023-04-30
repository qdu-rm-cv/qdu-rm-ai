#include "raspi_camera.hpp"

void RaspiCamera::GrabPrepare() { return; }

void RaspiCamera::GrabLoop() {
  bool err = false;
  cam_ >> frame_;
  err = frame_.empty();
  if (!err) {
    std::lock_guard<std::mutex> lock(frame_stack_mutex_);
    frame_stack_.push_front(frame_);
    frame_signal_.Signal();
  } else {
    SPDLOG_WARN("Empty frame");
  }
}

void RaspiCamera::PublishLoop() {
  if (!frame_stack_.empty()) {
    cam_topic_.Publish(frame_stack_.front());
  }
}

bool RaspiCamera::OpenPrepare(unsigned int index) {
  bool err = false;
  cam_.open(index);
  err = cam_.isOpened();
  if (err) {
    SPDLOG_WARN("Open device : {}", index);
    SPDLOG_WARN("FPS : {}", cam_.get(cv::CAP_PROP_FPS));
  } else {
    SPDLOG_DEBUG("No device : {}", index);
  }
  return err;
}

/**
 * @brief Construct a new RaspiCamera object
 *
 */
RaspiCamera::RaspiCamera() { SPDLOG_TRACE("Constructed."); }

/**
 * @brief Construct a new RaspiCamera object
 *
 * @param index 相机索引号
 * @param height 输出图像高度
 * @param width 输出图像宽度
 */
RaspiCamera::RaspiCamera(unsigned int index, unsigned int height,
                         unsigned int width) {
  Open(index);
  Setup(height, width);
  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Destroy the RaspiCamera object
 *
 */
RaspiCamera::~RaspiCamera() {
  Close();
  SPDLOG_TRACE("Destructed.");
}

/**
 * @brief 设置相机参数
 *
 * @param height 输出图像高度
 * @param width 输出图像宽度
 */
void RaspiCamera::Setup(unsigned int height, unsigned int width) {
  Camera::Setup(height, width);
  cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cam_.set(cv::CAP_PROP_FRAME_WIDTH, width);
}

/**
 * @brief 关闭相机设备
 *
 * @return int 状态代码
 */
int RaspiCamera::Close() {
  grabing = false;
  grab_thread_.join();
  cam_.release();
  SPDLOG_DEBUG("Closed.");
  return EXIT_SUCCESS;
}

/**
 * @brief 相机标定
 *
 */
void RaspiCamera::Calibrate() {}
