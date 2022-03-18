#include "behavior.hpp"

#include <random>

#include "spdlog/spdlog.h"

Behavior::Behavior() {
  status_.low_hp = false;
  status_.bullet_empty = false;
  status_.under_attack = false;
  last_base_hp_ = 3000;
  SPDLOG_TRACE("Constructed.");
}

Behavior::Behavior(const bool &low_hp, const bool &under_attack,
                   const bool &bullet_empty) {
  status_.low_hp = low_hp;
  status_.under_attack = under_attack;
  status_.bullet_empty = bullet_empty;
  last_base_hp_ = 0;
  SPDLOG_TRACE("Constructed.");
}

Behavior::~Behavior() { SPDLOG_TRACE("Destructed."); }

void Behavior::Update(int base_hp, int sentry_hp, int bullet_num) {
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
  static std::default_random_engine engine(time(NULL));
  static std::uniform_real_distribution<float> distributer(3.0f, 5.0f);

  data_.chassis_move_vec.vy =
      (v == 0 ? 1 : v / std::abs(v)) * distributer(engine);
  if (status_.low_hp || status_.bullet_empty) {
    data_.chassis_move_vec.vy *= 2;
  }
  if (status_.under_attack)
    data_.chassis_move_vec.wz = M_2_PI / 3;
  else
    data_.chassis_move_vec.wz = M_PI / 3;
}
void Behavior::SetNotice(game::Alert alert) {
  if (alert.enemy_buff) data_.notice |= AI_NOTICE_BUFF;
  if (alert.enemy_snipe) data_.notice |= AI_NOTICE_SNIPE;
  if (alert.enemy_slope) data_.notice |= AI_NOTICE_SLOPE;
  if (alert.self_outpost) data_.notice |= AI_NOTICE_OUTPOST;
  if (alert.self_sentry) data_.notice |= AI_NOTICE_SENTRY;
  if (alert.self_base) data_.notice |= AI_NOTICE_BASE;
  SPDLOG_DEBUG("Notice has been set.");
}

Protocol_DownData_t &Behavior::GetData() { return data_; }
