
#include "armor_ui.hpp"
#include "buff_ui.hpp"
#include "common.hpp"
#include "light_ui.hpp"
#include "opencv2/opencv.hpp"

namespace {

const std::string kLOG = "logs/ui_param.log";
const std::string kPARAMARMOR = "../../../../runtime/RMUL2022_Armor.json";
const std::string kPARAMBUFF = "../../../../runtime/RMUT2022_Buff.json";
const std::string kPARAMLIGHT = "../../../../runtime/RMUT2022_Light.json";
const std::string kWINDOW = "ui_setting";

/* 此处修改按钮数量 */
const int kB_NUM = 3;

const int kB_H = 52;
const int kW_O = 0;
const int kW_W = 100; /* 窗口宽度，建议偶数 */
constexpr int kW_H = kB_H * kB_NUM;

const int kF_FACE = cv::FONT_HERSHEY_COMPLEX;
const double kF_SCALE = 0.7;
const int kF_THICK = 1;

component::AimMethod method = component::AimMethod::kUNKNOWN;

}  // namespace

void MouseCallback(int event, int x, int y, int, void*);

void CreateButton(cv::Mat frame, const std::string& name, const int& pos);

int main(int argc, char const* argv[]) {
  (void)argc;
  (void)argv;

  cv::namedWindow(kWINDOW);
  cv::Mat black(kW_H, kW_W, CV_8UC3, cv::Scalar(0, 0, 0));

  CreateButton(black, "Armor", 1);
  CreateButton(black, "Buff", 2);
  CreateButton(black, "Light", 3);

  cv::imshow(kWINDOW, black);
  cv::setMouseCallback(kWINDOW, MouseCallback);
  cv::waitKey(0);

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

void MouseCallback(int event, int x, int y, int, void*) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    SPDLOG_INFO("x:{}, y:{}, {} th button", x, y, (y / kB_H) + 1);
    switch (y / kB_H) {
      case 0:
        method = component::AimMethod::kARMOR;
        break;
      case 1:
        method = component::AimMethod::kBUFF;
        break;
      case 2:
        method = component::AimMethod::kLIGHT;
        break;
      default:
        method = component::AimMethod::kUNKNOWN;
        break;
    }

    SPDLOG_INFO("{} now, press any key to continue...",
                component::AimMethodToString(method));
  }
  return;
}

void CreateButton(cv::Mat frame, const std::string& name, const int& pos) {
  if (pos > kB_NUM) {
    SPDLOG_ERROR("button num({}) is out of range({})", pos, kB_NUM);
    return;
  }

  int top = (pos - 1) * kB_H + 3; /* 收缩边框，故使top += 3 */
  int bottom = pos * kB_H - 3;    /* 收缩边框，故使bottom -= 3 */

  int baseline;
  cv::Size size = cv::getTextSize(name, kF_FACE, kF_SCALE, kF_THICK, &baseline);
  cv::Point pt(kW_W / 2 - size.width / 2, kB_H * (pos - 0.5) + size.height / 2);
  cv::putText(frame, name, pt, kF_FACE, kF_SCALE, draw::kGREEN, kF_THICK);

  std::vector<cv::Point> pts = {
      {kW_O, top}, {kW_W - 1, top}, {kW_W - 1, bottom}, {kW_O, bottom}};
  for (int i = 0; i < 4; ++i)
    cv::line(frame, pts[i], pts[(i + 1) % 4], draw::kGREEN, 2);
}
