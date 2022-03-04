
#include "armor_ui.hpp"
#include "buff_ui.hpp"
#include "common.hpp"
#include "light_ui.hpp"

namespace {

const std::string kLOG = "logs/ui_param.log";
const std::string kPARAMARMOR = "../../../../runtime/RMUL2022_Armor.json";
const std::string kPARAMBUFF = "../../../../runtime/RMUT2022_Buff.json";
const std::string kPARAMLIGHT = "../../../../runtime/RMUT2022_Light.json";
const std::string kWINDOW = "ui_setting";

}  // namespace

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  std::string name;
  std::cin >> name;
  component::AimMethod method = component::StringToAimMethod(name);

  if (method == component::AimMethod::kARMOR) {
    ArmorUIParam ui_param(kLOG, kPARAMARMOR);
    ui_param.Run();
  } else if (method == component::AimMethod::kBUFF) {
    BuffUIParam ui_param(kLOG, kPARAMBUFF);
    ui_param.Run();
  } else if (method == component::AimMethod::kLIGHT) {
    LightUIParam ui_param(kLOG, kPARAMLIGHT);
    ui_param.Run();
  }

  return EXIT_SUCCESS;
}
