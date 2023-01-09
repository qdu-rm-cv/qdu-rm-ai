#include "servo.hpp"

#include "spdlog/spdlog.h"

/**
 * @brief Construct a new Servo object
 *
 */
Servo::Servo() { SPDLOG_TRACE("Constructed."); }

/**
 * @brief Construct a new Servo object
 *
 * @param dev_path 具体要读写的舵机设备
 */
Servo::Servo(const std::string& dev_path) {
  Open(dev_path);
  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Destroy the Servo object
 *
 */
Servo::~Servo() { SPDLOG_TRACE("Destructed."); }

/**
 * @brief 打开舵机
 *
 * @param dev_path 具体要读写的舵机设备
 */
void Servo::Open(const std::string& dev_path) {
  (void)dev_path;
  SPDLOG_DEBUG("Servo {} has been opened", dev_path);
}

/**
 * @brief 检查舵机是否打开
 *
 * @return true 已打开
 * @return false 未打开
 */
bool Servo::IsOpen() {
  if (dev_)
    return true;
  else
    return false;
}

/**
 * @brief 配置舵机
 *
 * @param hi_width 最大角度时的占空时间
 * @param lo_width 最小角度时的占空时间
 * @param hi_angle 最大角度
 * @param lo_angle 最小角度
 * @return true 成功
 * @return false 失败
 */
bool Servo::Config(float hi_width, float lo_width, float hi_angle,
                   float lo_angle) {
  (void)hi_width;
  (void)lo_width;
  (void)hi_angle;
  (void)lo_angle;
  SPDLOG_DEBUG("Configure Success.");
  return true;
}

/**
 * @brief
 *
 * @param angle 转到的角度
 * @return true 成功
 * @return false 是失败
 */
bool Servo::Set(float angle) {
  (void)angle;
  return true;
}

/**
 * @brief 关闭
 *
 * @return int 状态代码
 */
int Servo::Close() { return 0; }
