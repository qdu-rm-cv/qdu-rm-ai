#include "behavior.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "common.hpp"
#include "gtest/gtest.h"
#include "node.hpp"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/spdlog.h"

namespace {

static const char *const xml_tree_test = R"(

<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
    <Fallback name="root_sequence">
        <LowHP name="low_hp"/>
        <Sequence name="fight">
            <EnamyVisable name="enamy_visable"/>
            <Track name="track"/>
            <Attack name="attack"/>
        </Sequence>
    </Fallback>
    </BehaviorTree>
</root>
 
)";

}

TEST(TestBehavior, TestSimpleTree) {
  BT::BehaviorTreeFactory factory;

  RegisterNode(factory);

  auto tree = factory.createTreeFromText(xml_tree_test);

  tree.tickRoot();
}

TEST(TestBehavior, TestWithoutTree) {
  RMlogger::SetLogger(spdlog::level::level_enum::warn);
  Behavior manager(false, false, false);
  manager.Init(300, 400, 200);
  component::Euler elur = {1, 1, 1};
  for (int i = 0; i < 5; i++) {
    manager.Aim(elur);
    manager.Move(3);
    Protocol_DownData_t data = manager.GetData();
    float vx = data.chassis_move_vec.vx;
    float vy = data.chassis_move_vec.vy;
    float wz = data.chassis_move_vec.wz;
    float pitch = data.gimbal.pit;
    float yaw = data.gimbal.yaw;
    float roll = data.gimbal.rol;
    SPDLOG_WARN("{0:b}", (unsigned int)data.notice);
    SPDLOG_WARN("vx : {}, vy : {}, wz : {}", vx, vy, wz);
    SPDLOG_WARN("pit : {}, yaw : {}, rol : {}", pitch, yaw, roll);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
