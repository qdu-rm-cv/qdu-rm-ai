#include "compensator.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kG = 9.7945;
const double kINFANTRY = 60.;
const double kHERO = 10.;
const double kSENTRY = 10.;
const double kBIG_ARMOR = 230. / 127 * cos(15. / 180 * M_PI);
const double kSMALL_ARMOR = 135. / 125 * cos(15. / 180 * M_PI);

}  // namespace

void Compensator::VisualizePnp(Armor& armor, const cv::Mat& output,
                               bool add_lable) {
  std::vector<cv::Point2f> out_points;
  cv::projectPoints(armor.ImageVertices(), armor.GetRotVec(),
                    armor.GetTransVec(), cam_mat_, distor_coff_, out_points);
  for (std::size_t i = 0; i < out_points.size(); ++i) {
    cv::line(output, out_points[i], out_points[(i + 1) % out_points.size()],
             draw::kBLACK);
  }
  if (add_lable) {
    cv::putText(output, "PNP", out_points[0], draw::kCV_FONT, 1.0,
                draw::kYELLOW);
  }
}

Compensator::Compensator() { SPDLOG_TRACE("Constructed."); }

Compensator::Compensator(const std::string& cam_mat_path,
                         const game::Arm& arm) {
  SPDLOG_TRACE("Constructed.");
  // TODO(RX.Jiang) : 和机械、兵种相关，后期放到namespace
  SetArm(arm);
  LoadCameraMat(cam_mat_path);
}

Compensator::~Compensator() { SPDLOG_TRACE("Destructed."); }

void Compensator::SetArm(const game::Arm& arm) {
  arm_ = arm;
  if (arm == game::Arm::kINFANTRY)
    gun_cam_distance_ = kINFANTRY;
  else if (arm == game::Arm::kHERO)
    gun_cam_distance_ = kHERO;
  else if (arm == game::Arm::kSENTRY)
    gun_cam_distance_ = kSENTRY;
}

void Compensator::LoadCameraMat(const std::string& path) {
  cv::FileStorage fs(path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);

  if (fs.isOpened()) {
    cam_mat_ = fs["cam_mat"].mat();
    distor_coff_ = fs["distor_coff"].mat();
    if (cam_mat_.empty() && distor_coff_.empty()) {
      SPDLOG_ERROR("Can not load cali data.");
    } else {
      SPDLOG_DEBUG("Loaded cali data.");
    }
  } else {
    SPDLOG_ERROR("Can not open file: '{}'", path);
  }
}

void Compensator::PnpEstimate(Armor& armor) {
  cv::Mat rot_vec, trans_vec;
  std::vector<cv::Point2f> trsd_cords(4);  // Points of 2D after update
  /*调整识别到的像素坐标,手动消除与处理带来的2D坐标
  不准的为问题,该参数可以根据ui_param灯条的变形情况
  来确定*/
  double k = 1.15;
  cv::Point2f t1 =
      (armor.ImageVertices()[1] - armor.ImageVertices()[0]) * k;  // 向量t1,t2
  cv::Point2f t2 = (armor.ImageVertices()[2] - armor.ImageVertices()[3]) * k;
  cv::Point2f br = armor.ImageVertices()[0];  // Right bottom point
  cv::Point2f bl = armor.ImageVertices()[3];  // Left bottom point

  // Points of 2D after adjusted
  std::vector<cv::Point2f> ori_cords = {br, br + t1, bl + t2, bl};

  double k2;  // k2值是目标装甲板的长宽比
  if (armor.GetArmorType() == LARGE) {
    k2 = kBIG_ARMOR;
  } else if (armor.GetModel() == game::Model::kBUFF) {
    k2 = 1;  // TODO(GY.Wang): 等Buff和buff_detector完成后修改
    SPDLOG_ERROR("Error param of buff has not been set!");
    return;
  } else {
    k2 = kSMALL_ARMOR;
  }

  UpdateImgPoints(ori_cords, k2, trsd_cords);
  real_img_ratio_ = 125. / cv::norm(trsd_cords[0] - trsd_cords[1]);

  auto new_img_center =
      (trsd_cords[0] + trsd_cords[1] + trsd_cords[2] + trsd_cords[3]) / 4;
  // 重构之后装甲板的中心会有偏移
  double center_diff_x = abs(armor.ImageCenter().x - new_img_center.x) *
                         real_img_ratio_;  //重构之后装甲板的中心会有偏移
  double center_diff_y =
      abs(armor.ImageCenter().y - new_img_center.y) * real_img_ratio_;

  cv::solvePnP(armor.PhysicVertices(),
               /* armor.ImageVertices() */ trsd_cords, cam_mat_, distor_coff_,
               rot_vec, trans_vec, false, cv::SOLVEPNP_ITERATIVE);
  trans_vec.at<double>(0, 0) -= center_diff_x;
  trans_vec.at<double>(1, 0) -= center_diff_y;

  trans_vec.at<double>(1, 0) -= gun_cam_distance_;
  armor.SetRotVec(rot_vec), armor.SetTransVec(trans_vec);
}

void Compensator::SolveAngles(Armor& armor, const component::Euler& euler) {
  component::Euler aiming_eulr;
  PnpEstimate(armor);
  double x_pos = armor.GetTransVec().at<double>(0, 0);
  double y_pos = armor.GetTransVec().at<double>(1, 0);
  double z_pos = armor.GetTransVec().at<double>(2, 0);
  SPDLOG_INFO("initial pitch : {}, initial yaw : {}", euler.pitch, euler.yaw);
  SPDLOG_WARN("x : {}, y : {}, z : {} ", x_pos, y_pos, z_pos);
  distance_ = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos) / 1000;
  SPDLOG_INFO("distance:{}", distance_);

  if (distance_ > 5) {
    // PinHoleSolver
    double ax = cam_mat_.at<double>(0, 0);
    double ay = cam_mat_.at<double>(1, 1);
    double u0 = cam_mat_.at<double>(0, 2);
    double v0 = cam_mat_.at<double>(1, 2);

    std::vector<cv::Point2f> out;
    cv::undistortPoints(std::vector<cv::Point2f>{armor.image_center_}, out,
                        cam_mat_, distor_coff_, cv::noArray(), cam_mat_);
    aiming_eulr.pitch = -atan((out.front().y - v0) / ay);
    aiming_eulr.yaw = atan((out.front().x - u0) / ax);
  } else {
    // P4PSolver
    aiming_eulr.pitch = -atan(y_pos / sqrt(x_pos * x_pos + z_pos * z_pos));
    aiming_eulr.yaw = atan(x_pos / z_pos);
  }
  SPDLOG_INFO("compensator pitch : {}", aiming_eulr.pitch);
  aiming_eulr.pitch = aiming_eulr.pitch + euler.pitch;
  aiming_eulr.yaw = aiming_eulr.yaw + euler.yaw;

  SPDLOG_INFO("final pitch : {}", aiming_eulr.pitch);
  armor.SetAimEuler(aiming_eulr);
}

void Compensator::Apply(tbb::concurrent_vector<Armor>& armors,
                        const double ballet_speed,
                        const component::Euler& euler, game::AimMethod method) {
#if 0
  cv::Point2f frame_center(kIMAGE_WIDTH / 2, kIMAGE_HEIGHT / 2);
  std::sort(armors.begin(), armors.end(),
            [frame_center](Armor& armor1, Armor& armor2) {
              return cv::norm(armor1.ImageCenter() - frame_center) <
                     cv::norm(armor2.ImageCenter() - frame_center);
            });
#endif
  std::sort(armors.begin(), armors.end(), [](Armor& a, Armor& b) {
    if (a.GetArea() > b.GetArea()) {
      return false;
    } else {
      return abs(a.ImageCenter().x - kIMAGE_WIDTH / 2) <=
             abs(b.ImageCenter().x - kIMAGE_WIDTH / 2);
    }
  });
  auto& armor = armors.front();
  if (armor.GetModel() == game::Model::kUNKNOWN) {
    armor.SetModel(game::Model::kINFANTRY);
    SPDLOG_ERROR("Hasn't set model.");
  }
  SolveAngles(armor, euler);
  CompensateGravity(armor, ballet_speed, method);
}

void Compensator::Apply(Armor& armor, const double ballet_speed,
                        const component::Euler& euler, game::AimMethod method) {
  cv::Point2f frame_center(kIMAGE_WIDTH / 2, kIMAGE_HEIGHT / 2);

  if (armor.GetModel() == game::Model::kUNKNOWN) {
    armor.SetModel(game::Model::kINFANTRY);
    SPDLOG_ERROR("Hasn't set model.");
  }
  SolveAngles(armor, euler);
  CompensateGravity(armor, ballet_speed, method);
}

void Compensator::VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                                  const cv::Mat& output, int verbose) {
  for (auto& armor : armors) {
    VisualizePnp(armor, output, verbose > 1);
  }
}

void Compensator::CompensateGravity(Armor& armor, const double ballet_speed,
                                    game::AimMethod method) {
  //高斯牛顿迭代法，水平方向阻力模型
  if (method == game::AimMethod::kARMOR || method == game::AimMethod::kBUFF) {
    component::Euler aiming_eulr = armor.GetAimEuler();
    SPDLOG_INFO("初始值 {}", aiming_eulr.pitch);
    SPDLOG_INFO("Distance: {}", distance_);
    double x = distance_ * cos(aiming_eulr.pitch);
    double angle = aiming_eulr.pitch;
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    double k = 0 * real_img_ratio_;  //补偿高度
    double k0 = 0.1;                 //空气阻力系数
    double target_y = distance_ * sin(angle) + k;
    double temple_y = target_y;
    for (int i = 0; i < 10; i++) {
      double real_y;
      /*理想弹道模型
      double a = -kG * cos(angle) * cos(angle) * x * x /
                 (ballet_speed * ballet_speed * cos(angle) * cos(angle));
      double b = tan(angle) * cos(angle) * x;
      real_y = a + b;
      */
      double a = (exp(k0 * x) - 1) / (k0 * ballet_speed * sin(angle));
      real_y = ballet_speed * cos(angle) * a - (1 / 2 * kG * a * a);
      double diff = target_y - real_y;
      temple_y = temple_y + diff;
      if (distance_ > 5) {
        // PinHoleSolver
        double ay = cam_mat_.at<double>(1, 1);
        double v0 = cam_mat_.at<double>(1, 2);
        std::vector<cv::Point2f> in{
            cv::Point2f(armor.image_center_.x, temple_y)};
        std::vector<cv::Point2f> out;
        cv::undistortPoints(in, out, cam_mat_, distor_coff_, cv::noArray(),
                            cam_mat_);
        angle = -atan((out.front().y - v0) / ay);
      } else {
        double x_pos = armor.GetTransVec().at<double>(0, 0);
        double z_pos = armor.GetTransVec().at<double>(2, 0);
        // P4PSolver
        angle = -atan(temple_y / sqrt(x_pos * x_pos + z_pos * z_pos));
      }
      // SPDLOG_INFO("第{}次迭代", i);
      // SPDLOG_INFO("pitch: {}", angle);
    }
    if (1) {
      double a = cv::norm(armor.ImageVertices()[0] - armor.ImageVertices()[1]);
      double b = cv::norm(armor.ImageVertices()[2] - armor.ImageVertices()[3]);
      double c = std::max(a, b);
      double tan =
          abs(armor.ImageCenter().x - 640 / 2) * real_img_ratio_ / distance_;
      double add_yaw = atan(tan) / 180 * CV_PI;
      aiming_eulr.pitch = angle;
      aiming_eulr.yaw += add_yaw;
    }
    armor.SetAimEuler(aiming_eulr);
    SPDLOG_DEBUG("Armor Euler is setted");
  }
}
/**
 * @brief 更新矫正图像座标点
 *  图片坐标默认顺序: 左下，左上，右上，右下
 *  使用前体装甲板底边和书平面平行或者偏差不大并且者相机不能倾斜，以下函数中提到的
 *  length和width均为img中的,1/K为装甲版的实际长宽比，注意要区分大小装甲版
 *
 * @param origin_coords
 * @param k
 * @param transformed_coords
 */
void Compensator::UpdateImgPoints(
    std::vector<cv::Point2f>& origin_coords, double k,
    std::vector<cv::Point2f>& transformed_coords) {
  double length, width;
  width = std::max(cv::norm(origin_coords[0] - origin_coords[1]),
                   cv::norm(origin_coords[3] - origin_coords[2]));
  length = width * k;
  transformed_coords[0] = origin_coords[0];
  transformed_coords[1] = origin_coords[0] - cv::Point2f(0, width);
  transformed_coords[2] = transformed_coords[1] + cv::Point2f(length, 0);
  transformed_coords[3] = transformed_coords[2] + cv::Point2f(0, width);
}

#ifdef RMU2021
/**
 * @brief Angle θ required to hit coordinate (x, y)
 *
 * {\displaystyle \tan \theta ={\left({\frac {v^{2}\pm {\sqrt
 * {v^{4}-g(gx^{2}+2yv^{2})}}}{gx}}\right)}}
 *
 * @param target 目标坐标
 * @return double 出射角度
 */
double Compensator::SolveSurfaceLanchAngle(cv::Point2f target,
                                           double ballet_speed) {
  const double v_2 = pow(ballet_speed, 2);
  const double up_base =
      std::sqrt(std::pow(ballet_speed, 4) -
                kG * (kG * std::pow(target.x, 2) + 2 * target.y * v_2));
  const double low = kG * target.x;
  const double ans1 = std::atan2(v_2 + up_base, low);
  const double ans2 = std::atan2(v_2 - up_base, low);

  if (std::isnan(ans1)) return std::isnan(ans2) ? 0. : ans2;
  if (std::isnan(ans2)) return std::isnan(ans1) ? 0. : ans1;
  return std::min(ans1, ans2);
}

cv::Vec3f Compensator::EstimateWorldCoord(Armor& armor) {
  cv::Mat rot_vec, trans_vec;
  cv::solvePnP(armor.PhysicVertices(), armor.ImageVertices(), cam_mat_,
               distor_coff_, rot_vec, trans_vec, false, cv::SOLVEPNP_ITERATIVE);
  armor.SetRotVec(rot_vec), armor.SetTransVec(trans_vec);
  cv::Mat world_coord =
      ((cv::Vec2f(armor.ImageCenter()) * cam_mat_.inv() - trans_vec) *
       armor.GetRotMat().inv());
  return cv::Vec3f(world_coord);
}
#endif
