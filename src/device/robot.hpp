#pragma once

#include <mutex>
#include <queue>
#include <thread>

#include "common.hpp"
#include "crc16.hpp"
#include "opencv2/opencv.hpp"
#include "protocol.h"
#include "serial.hpp"

class Robot {
 private:
  Serial serial_;
  bool thread_continue = false;
  std::thread thread_recv_, thread_trans_;

  std::queue<Protocol_DownData_t> commandq_;
  Protocol_UpDataReferee_t ref_;
  Protocol_UpDataMCU_t mcu_;

  std::mutex mutex_commandq_, mutex_ref_, mutex_mcu_;

  void ThreadRecv();
  void ThreadTrans();

 public:
  Robot();
  Robot(const std::string &dev_path);
  ~Robot();

  void Init(const std::string &dev_path);

  game::Team GetEnemyTeam();
  game::Race GetRace();
  double GetTime();
  game::RFID GetRFID();
  int GetBaseHP();
  int GetSentryHP();
  int GetBalletRemain();

  game::Arm GetArm();
  cv::Mat GetRotMat();
  float GetBalletSpeed();
  float GetChassicSpeed();

  void Pack(Protocol_DownData_t &data, const double distance);
};
