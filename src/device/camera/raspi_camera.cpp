#include "raspi_camera.hpp"

void RaspiCamera::GrabPrepare() { return; }

void RaspiCamera::GrabLoop() {
  cv::Mat frame;
  cam_ >> frame;
  if (!frame.empty()) {
    std::lock_guard<std::mutex> lock(frame_stack_mutex_);
    frame_stack_.push_front(frame.clone());
    frame_signal_.Give();
    SPDLOG_DEBUG("frame_stack_ size: {}", frame_stack_.size());
  } else {
    SPDLOG_WARN("Empty frame");
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
 * @param width 输出图像宽度
 * @param height 输出图像高度
 */
RaspiCamera::RaspiCamera(unsigned int index, unsigned int height,
                         unsigned int width) {
  Open(index);
  Setup(width, height);
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
 * @param width 输出图像宽度
 * @param height 输出图像高度
 */
void RaspiCamera::Setup(unsigned int width, unsigned int height) {
  Camera::Setup(width, height);
  cam_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cam_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
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
