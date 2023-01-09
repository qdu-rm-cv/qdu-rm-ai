#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace Action {

class Track : public BT::SyncActionNode {
 public:
  Track(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class Attack : public BT::SyncActionNode {
 public:
  Attack(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

class Dodge : public BT::SyncActionNode {
 public:
  Dodge(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};

}  // namespace Action

namespace Condition {

class EnamyVisable : public BT::ConditionNode {
 public:
  EnamyVisable(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

 private:
  bool is_visible_;
};

class LowHP : public BT::ConditionNode {
 public:
  LowHP(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

 private:
  bool is_low_HP_;
};

class UnderAttack : public BT::ConditionNode {
 public:
  UnderAttack(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

 private:
  bool is_under_attack_;
};

class NoAmmo : public BT::ConditionNode {
 public:
  NoAmmo(const std::string &name, const BT::NodeConfiguration &node_cfg);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
}  // namespace Condition

void RegisterNode(BT::BehaviorTreeFactory &factory);

// TODO(YZ.Hou, ZX.Song) : 完善补充行为树模块
