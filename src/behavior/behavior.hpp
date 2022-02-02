#pragma once

#include "common.hpp"
#include "protocol.h"

struct NodeStatus {
  bool low_hp;
  bool under_attack;
  bool bullet_empty;
};

class Behavior {
 private:
  NodeStatus status_;
  int last_base_hp_;
  Protocol_DownData_t data_;

 public:
  Behavior();
  Behavior(const bool &low_hp, const bool &under_attack,
           const bool &bullet_empty);
  ~Behavior();

  void Init(int base_hp, int sentry_hp, int bullet_num);
  void Aim(component::Euler aiming_eulr);
  void Move(float v);

  const Protocol_DownData_t GetData() const;
};
