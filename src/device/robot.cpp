#include "robot.hpp"

#include "spdlog/spdlog.h"

namespace {

const double kFACTOR = 0.04;

}  // namespace

void Robot::ThreadRecv() {
  SPDLOG_DEBUG("[ThreadRecv] Started.");

  Protocol_ID_t id;
  Protocol_UpPackageReferee_t ref;
  Protocol_UpPackageMCU_t robot;

  while (thread_continue) {
    serial_.Recv(&id, sizeof(id));

    if (AI_ID_REF == id) {
      serial_.Recv(&ref, sizeof(ref));

      if (crc16::CRC16_Verify((uint8_t *)&ref, sizeof(ref))) {
        mutex_ref_.lock();
        std::memcpy(&ref_, &(ref.data), sizeof(ref_));
        mutex_ref_.unlock();
      }
    } else if (AI_ID_MCU == id) {
      serial_.Recv(&robot, sizeof(robot));

      if (crc16::CRC16_Verify((uint8_t *)&robot, sizeof(robot))) {
        mutex_mcu_.lock();
        std::memcpy(&mcu_, &(robot.data), sizeof(mcu_));
        mutex_mcu_.unlock();
      }
    }
  }
  SPDLOG_DEBUG("[ThreadRecv] Stoped.");
}

void Robot::ThreadTrans() {
  SPDLOG_DEBUG("[ThreadTrans] Started.");

  Protocol_DownPackage_t command;

  while (thread_continue) {
    pack_signal_.Wait();
    bool is_empty = true;
    mutex_command_.lock();
    if (commandq_.size() > 0) {
      command.data = commandq_.front();
      commandq_.pop_front();
      is_empty = false;
    }
    mutex_command_.unlock();
    if (!is_empty) {
      command.crc16 = crc16::CRC16_Calc((uint8_t *)&command.data,
                                        sizeof(command.data), UINT16_MAX);
      if (serial_.Trans((char *)&command, sizeof(command))) {
        mutex_command_.lock();
        while (!serial_.Reopen())
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        mutex_command_.unlock();
      }
    }
  }
  SPDLOG_DEBUG("[ThreadTrans] Stoped.");
}

Robot::Robot() { SPDLOG_TRACE("Constructed."); }

Robot::Robot(const std::string &dev_path) {
  Init(dev_path);

  SPDLOG_TRACE("Constructed.");
}

Robot::~Robot() {
  serial_.Close();

  thread_continue = false;
  thread_recv_.join();
  thread_trans_.join();
  SPDLOG_TRACE("Destructed.");
}

void Robot::Init(const std::string &dev_path) {
  serial_.Open(dev_path);

  thread_continue = true;
  thread_recv_ = std::thread(&Robot::ThreadRecv, this);
  thread_trans_ = std::thread(&Robot::ThreadTrans, this);
}

void Robot::Init() {
  serial_.Open();

  thread_continue = true;
  thread_recv_ = std::thread(&Robot::ThreadRecv, this);
  thread_trans_ = std::thread(&Robot::ThreadTrans, this);
}

game::Team Robot::GetEnemyTeam() {
  if (ref_.team == AI_TEAM_RED)
    return game::Team::kBLUE;
  else if (ref_.team == AI_TEAM_BLUE)
    return game::Team::kRED;
  return game::Team::kUNKNOWN;
}

game::Race Robot::GetRace() {
  if (ref_.race == AI_RACE_RMUC)
    return game::Race::kRMUC;
  else if (ref_.race == AI_RACE_RMUT)
    return game::Race::kRMUT;
  else if (ref_.race == AI_RACE_RMUL1)
    return game::Race::kRMUL1;
  else if (ref_.race == AI_RACE_RMUL3)
    return game::Race::kRMUL3;
  return game::Race::kUNKNOWN;
}

double Robot::GetTime() { return ref_.time; }

game::RFID Robot::GetRFID() {
  if (ref_.rfid == AI_RFID_BUFF)
    return game::RFID::kBUFF;
  else if (ref_.rfid == AI_RFID_SNIP)
    return game::RFID::kSNIPE;
  return game::RFID::kUNKNOWN;
}

int Robot::GetBaseHP() { return ref_.base_hp; }

int Robot::GetSentryHP() { return ref_.sentry_hp; }

int Robot::GetBalletRemain() { return ref_.ballet_remain; }

game::Arm Robot::GetArm() {
  int num = 0;
  for (int i = 0; i < 6; i++) {
    num += ((ref_.arm >> (i)) & 0x01);
  }
  if (num != 1)
    return game::Arm::kUNKNOWN;
  else
    switch (ref_.arm & 0xFF) {
      case AI_ARM_INFANTRY:
        return game::Arm::kINFANTRY;
      case AI_ARM_HERO:
        return game::Arm::kHERO;
      case AI_ARM_ENGINEER:
        return game::Arm::kENGINEER;
      case AI_ARM_DRONE:
        return game::Arm::kDRONE;
      case AI_ARM_SENTRY:
        return game::Arm::kSENTRY;
      case AI_ARM_DART:
        return game::Arm::kDART;
      case AI_ARM_RADAR:
        return game::Arm::kRADAR;
      default:
        return game::Arm::kUNKNOWN;
    }
}

component::Euler Robot::GetEuler() {
  cv::Quatf q(mcu_.quat.q0, mcu_.quat.q1, mcu_.quat.q2, mcu_.quat.q3);
  cv::Vec3d vec = q.toEulerAngles(cv::QuatEnum::EulerAnglesType::INT_XYZ);
  component::Euler euler;
  euler.pitch = vec[0];
  euler.roll = vec[1];
  euler.yaw = vec[2];
  SPDLOG_DEBUG("P : {}, R : {}, Y : {}", euler.pitch, euler.roll, euler.yaw);
  return euler;
}

cv::Mat Robot::GetRotMat() {
  cv::Quatf q(mcu_.quat.q0, mcu_.quat.q1, mcu_.quat.q2, mcu_.quat.q3);
  return cv::Mat(q.toRotMat3x3(), true);
}

float Robot::GetBalletSpeed() { return mcu_.ball_speed; }

float Robot::GetChassicSpeed() { return mcu_.chassis_speed; }

void Robot::Pack(Protocol_DownData_t &data, double distance) {
  double w = mcu_.quat.q0, x = mcu_.quat.q1, y = mcu_.quat.q2, z = mcu_.quat.q3;
  component::Euler euler;

  const float sinr_cosp = 2.0f * (w * x + y * z);
  const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  euler.pitch = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (w * y - z * x);

  if (fabsf(sinp) >= 1.0f)
    euler.roll = copysignf(CV_PI / 2.0f, sinp);
  else
    euler.roll = asinf(sinp);

  const float siny_cosp = 2.0f * (w * z + x * y);
  const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  euler.yaw = atan2f(siny_cosp, cosy_cosp);

  data.notice &= ~AI_NOTICE_FIRE;
  if (fabs(euler.pitch - data.gimbal.pit) >= kFACTOR * distance)
    ;
  else if (fabs(euler.roll - data.gimbal.rol) >= kFACTOR * distance)
    ;
  else if (fabs(euler.yaw - data.gimbal.yaw) >= kFACTOR * distance)
    ;
  else
    data.notice |= AI_NOTICE_FIRE;

  mutex_command_.lock();
  commandq_.emplace_back(data);
  pack_signal_.Signal();
  mutex_command_.unlock();
}
