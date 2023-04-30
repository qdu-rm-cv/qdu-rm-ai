#include "hik_camera.hpp"

#include <cstring>
#include <exception>
#include <string>
#include <thread>

#include "common.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const unsigned int kSDK_VERSION = 50463230;

/**
 * @brief 检查HikRobot相机错误
 *
 * @param err
 * @param description
 * @param exit_flag
 *
 * @return true if err==MV_OK
 * @return false
 */
bool HikCheck(int err, const std::string_view description,
              bool exit_flag = true) {
  if (err != MV_OK) {
    if (exit_flag) {
      SPDLOG_ERROR("{} fail! err: {:#011x}", description, err + 128);
      exit(1);
    } else {
      SPDLOG_INFO("{}, code: {:#011x}", description, err);
    }

    return false;
  }
  return true;
}

}  // namespace

/**
 * @brief 打印设备信息
 *
 * @param mv_dev_info 设备信息结构体
 */
static void PrintDeviceInfo(MV_CC_DEVICE_INFO *mv_dev_info) {
  if (nullptr == mv_dev_info) {
    SPDLOG_ERROR("The Pointer of mv_dev_info is nullptr!");
    return;
  }
  if (mv_dev_info->nTLayerType == MV_USB_DEVICE) {
    SPDLOG_INFO("UserDefinedName: {}.",
                reinterpret_cast<char *>(
                    mv_dev_info->SpecialInfo.stUsb3VInfo.chUserDefinedName));

    SPDLOG_INFO("Serial Number: {}.",
                reinterpret_cast<char *>(
                    mv_dev_info->SpecialInfo.stUsb3VInfo.chSerialNumber));

    SPDLOG_INFO("Device Number: {}.",
                mv_dev_info->SpecialInfo.stUsb3VInfo.nDeviceNumber);
  } else {
    SPDLOG_WARN("Not supported.");
  }
}

void HikCamera::GrabPrepare() {
  std::memset(&raw_frame_, 0, sizeof(raw_frame_));
}

void HikCamera::GrabLoop() {
  if (HikCheck(MV_CC_GetImageBuffer(camera_handle_, &raw_frame_, 10000),
               "[GrabThread] GetImageBuffer", false)) {
    SPDLOG_DEBUG("[GrabThread] FrameNum: {}.",
                 raw_frame_.stFrameInfo.nFrameNum);
  }

  cv::Mat raw_mat(
      cv::Size(raw_frame_.stFrameInfo.nWidth, raw_frame_.stFrameInfo.nHeight),
      CV_8UC1, raw_frame_.pBufAddr);

  if (!raw_mat.empty()) cv::cvtColor(raw_mat, raw_mat, cv::COLOR_BayerRG2BGR);

  std::lock_guard<std::mutex> lock(frame_stack_mutex_);
  frame_stack_.clear();
  frame_stack_.push_front(raw_mat.clone());
  frame_signal_.Signal();
  SPDLOG_DEBUG("frame_stack_ size: {}", frame_stack_.size());
  if (nullptr != raw_frame_.pBufAddr) {
    HikCheck(MV_CC_FreeImageBuffer(camera_handle_, &raw_frame_),
             "[GrabThread] FreeImageBuffer");
  }
}

void HikCamera::PublishLoop() {
  if (!frame_stack_.empty()) {
    std::lock_guard<std::mutex> lock(frame_stack_mutex_);
    cam_topic_.Publish(frame_stack_.front());
    frame_signal_.Signal();
  }
}

bool HikCamera::OpenPrepare(unsigned int index) {
  SPDLOG_DEBUG("Open index: {}.", index);

  if (index >= mv_dev_list_.nDeviceNum) {
    SPDLOG_ERROR("Intput index:{} >= nDeviceNum:{} !", index,
                 mv_dev_list_.nDeviceNum);
    exit(-1);
    return false;
  }

  HikCheck(MV_CC_CreateHandle(&camera_handle_, mv_dev_list_.pDeviceInfo[0]),
           "CreateHandle");

  HikCheck(MV_CC_OpenDevice(camera_handle_), "OpenDevice", false);
  HikCheck(MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0), "TriggerMode");
  HikCheck(MV_CC_SetEnumValue(camera_handle_, "PixelFormat",
                              PixelType_Gvsp_BayerRG8),
           "PixelFormat");
  HikCheck(MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2),
           "AcquisitionMode");

  MVCC_FLOATVALUE frame_rate;

  if (HikCheck(MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate",
                                   &frame_rate),
               "ResultingFrameRate", false)) {
    SPDLOG_INFO("ResultingFrameRate: {}.", frame_rate.fCurValue);
  }

  HikCheck(MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0),
           "ExposureAuto closes");
  HikCheck(MV_CC_SetEnumValue(camera_handle_, "ExposureMode", 0),
           "ExposureMode");
  HikCheck(MV_CC_SetFloatValue(camera_handle_, "ExposureTime", 1000.0),
           "ExposureMode");

  unsigned int version = MV_CC_GetSDKVersion();
  SPDLOG_INFO("SDK version : {}", version);
  if (version <= kSDK_VERSION) {
    HikCheck((MV_CC_SetEnumValue(camera_handle_, "GammaSelector", 1)),
             "GammaSelector");
    HikCheck(MV_CC_SetBoolValue(camera_handle_, "GammaEnable", true),
             "GammaEnable");
  }

  HikCheck(MV_CC_StartGrabbing(camera_handle_), "StartGrabbing");
  return true;
}

/**
 * @brief 相机初始化前的准备工作
 *
 */
void HikCamera::Prepare() {
  SPDLOG_DEBUG("Prepare.");

  std::memset(&mv_dev_list_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  HikCheck(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &mv_dev_list_),
           "EnumDevices");

  if (mv_dev_list_.nDeviceNum > 0) {
    for (unsigned int i = 0; i < mv_dev_list_.nDeviceNum; ++i) {
      SPDLOG_INFO("Device {} slected.", i);
      MV_CC_DEVICE_INFO *dev_info = mv_dev_list_.pDeviceInfo[i];
      if (dev_info == nullptr) {
        SPDLOG_ERROR("Error Reading dev_info");
      } else {
        PrintDeviceInfo(dev_info);
      }
    }
  } else {
    SPDLOG_ERROR("Find No Devices!");
  }

  auto cam_callback = [](cv::Mat &frame) {
    // TODO(ZX.Song) : to be continue...
  };
}

/**
 * @brief Construct a new HikCamera object
 *
 */
HikCamera::HikCamera() {
  Prepare();
  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Construct a new HikCamera object
 *
 * @param index 相机索引号
 * @param height 输出图像高度
 * @param width 输出图像宽度
 */
HikCamera::HikCamera(unsigned int index, unsigned int height,
                     unsigned int width) {
  Prepare();
  Open(index);
  Setup(height, width);
  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Destroy the HikCamera object
 *
 */
HikCamera::~HikCamera() {
  Close();
  SPDLOG_TRACE("Destructed.");
}

/**
 * @brief 关闭相机设备
 *
 * @return int 状态代码
 */
int HikCamera::Close() {
  grabing = false;
  grab_thread_.join();
  topic_thread_.join();

  HikCheck(MV_CC_StopGrabbing(camera_handle_), "StopGrabbing");
  HikCheck(MV_CC_CloseDevice(camera_handle_), "CloseDevice");
  HikCheck(MV_CC_DestroyHandle(camera_handle_), "DestroyHandle");
  SPDLOG_DEBUG("Closed.");

  return MV_OK;
}

/**
 * @brief 相机标定
 *
 */
void Calibrate() {
  // TODO(GY.Wang,BL.Feng) : calibration
  return;
}
