#include "behavior.hpp"

#include <random>

#include "spdlog/spdlog.h"

Behavior::Behavior() { SPDLOG_TRACE("Constructed."); }

Behavior::Behavior(const bool &low_hp, const bool &under_attack,
                   const bool &bullet_empty) {
  status_.low_hp = low_hp;
  status_.under_attack = under_attack;
  status_.bullet_empty = bullet_empty;
  last_base_hp_ = 0;
  SPDLOG_TRACE("Constructed.");
}

Behavior::~Behavior() { SPDLOG_TRACE("Destructed."); }

void Behavior::Init(int base_hp, int sentry_hp, int bullet_num) {
  if (sentry_hp < 100)
    status_.low_hp = true;
  else
    status_.low_hp = false;

  if (base_hp < last_base_hp_)
    status_.under_attack = true;
  else
    status_.under_attack = false;
  last_base_hp_ = base_hp;

  status_.bullet_empty = (bullet_num == 0);
}

void Behavior::Aim(component::Euler aiming_eulr) {
  data_.notice &= 0x00;
  data_.gimbal.pit = 0;
  data_.gimbal.yaw = 0;
  data_.gimbal.rol = 0;
  data_.chassis_move_vec.vx = 0;
  data_.chassis_move_vec.vy = 0;
  data_.chassis_move_vec.wz = 0;
  if (!status_.bullet_empty) {
    data_.gimbal.pit = aiming_eulr.pitch;
    data_.gimbal.yaw = aiming_eulr.yaw;
    data_.gimbal.rol = aiming_eulr.roll;
  }
}

void Behavior::Move(float v) {
  std::default_random_engine e(time(NULL));
  std::uniform_real_distribution<float> u(3.0f, 5.0f);
  // TODO : random 修改毫秒级播种函数
  data_.chassis_move_vec.vy = u(e) * v / std::abs(v);
  if (status_.low_hp || status_.bullet_empty) {
    data_.chassis_move_vec.vy *= 2;
  }
  if (status_.under_attack)
    data_.chassis_move_vec.wz = M_2_PI / 3;
  else
    data_.chassis_move_vec.wz = M_PI / 3;
}

const Protocol_DownData_t Behavior::GetData() const { return data_; }
