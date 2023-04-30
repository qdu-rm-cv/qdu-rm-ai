#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include "camera.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/opencv.hpp"

class RaspiCamera : public Camera {
 private:
  cv::VideoCapture cam_;
  cv::Mat frame_ = cv::Mat(cv::Size(640, 480), CV_16UC3);

  void GrabPrepare();
  void GrabLoop();
  void PublishLoop();
  bool OpenPrepare(unsigned int index);

 public:
  /**
   * @brief Construct a new RaspiCamera object
   *
   */
  RaspiCamera();

  /**
   * @brief Construct a new RaspiCamera object
   *
   * @param index 相机索引号
   * @param height 输出图像高度
   * @param width 输出图像宽度
   */
  RaspiCamera(unsigned int index, unsigned int height, unsigned int width);

  /**
   * @brief Destroy the RaspiCamera object
   *
   */
  ~RaspiCamera();

  /**
   * @brief 设置相机参数
   *
   * @param height 输出图像高度
   * @param width 输出图像宽度
   */
  void Setup(unsigned int height, unsigned int width);

  /**
   * @brief 关闭相机设备
   *
   * @return int 状态代码
   */
  int Close();

  /**
   * @brief 相机标定
   *
   */
  void Calibrate();
};
