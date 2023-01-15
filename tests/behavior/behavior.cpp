#include "behavior.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gtest/gtest.h"
#include "log.hpp"
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

}  // namespace

TEST(TestBehavior, TestSimpleTree) {
  BT::BehaviorTreeFactory factory;

  RegisterNode(factory);

  auto tree = factory.createTreeFromText(xml_tree_test);

  tree.tickRoot();
}

TEST(TestBehavior, TestWithoutTree) {
  component::Logger::SetLogger();
  Behavior manager(false, false, false);
  manager.Update(300, 400, 200);
  component::Euler elur(1, 1, 1);
  for (int i = 0; i < 5; i++) {
    manager.Aim(elur);
    manager.Move(i);
    Protocol_DownData_t data = manager.GetData();
    // hex: {0:x};  oct: {0:o};  bin: {0:b}
    SPDLOG_WARN("{0:b}", static_cast<unsigned int>(data.notice));
    SPDLOG_WARN("vx : {}, vy : {}, wz : {}",
                static_cast<float>(data.chassis_move_vec.vx),
                static_cast<float>(data.chassis_move_vec.vy),
                static_cast<float>(data.chassis_move_vec.wz));
    SPDLOG_WARN("pit : {}, yaw : {}, rol : {}",
                static_cast<float>(data.gimbal.pit),
                static_cast<float>(data.gimbal.yaw),
                static_cast<float>(data.gimbal.rol));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
