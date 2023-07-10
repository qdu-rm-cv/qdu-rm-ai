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

  real_img_ratio_ = 125. / cv::norm(trsd_cords[0] - trsd_cords[1]);  // mm/px

  auto new_img_center =
      (trsd_cords[0] + trsd_cords[1] + trsd_cords[2] + trsd_cords[3]) / 4;
  // 重构之后装甲板的中心会有偏移
  double center_diff_x = abs(armor.ImageCenter().x - new_img_center.x) *
                         real_img_ratio_;  //重构之后装甲板的中心会有偏移
  double center_diff_y =
      abs(armor.ImageCenter().y - new_img_center.y) * real_img_ratio_;

  cv::solvePnP(armor.PhysicVertices(), armor.ImageVertices(),
               cam_mat_, distor_coff_, rot_vec, trans_vec, false,
               cv::SOLVEPNP_ITERATIVE);
  //TODO(SOMRBODY):Solver the problem of accency in x and y;
  trans_vec.at<double>(1, 0) -= gun_cam_distance_;
  armor.SetRotVec(rot_vec);
  armor.SetTransVec(trans_vec);
}

void Compensator::SolveAngles(Armor& armor, const component::Euler& euler) {
  component::Euler aiming_eulr;
  PnpEstimate(armor);
  double x_pos = armor.GetTransVec().at<double>(0, 0);
  double y_pos = armor.GetTransVec().at<double>(1, 0);
  double z_pos = armor.GetTransVec().at<double>(2, 0);
  SPDLOG_INFO("initial pitch : {}, initial yaw : {}", euler.pitch, euler.yaw);
  SPDLOG_WARN("x : {}, y : {}, z : {} ", x_pos, y_pos, z_pos);
  distance_ = sqrt(z_pos * z_pos) / 1000;
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
    aiming_eulr.pitch =
        -atan(y_pos / sqrt(x_pos * x_pos + z_pos * z_pos));
    aiming_eulr.yaw = atan(x_pos / z_pos);
  }
  SPDLOG_INFO("compensator pitch : {}", aiming_eulr.pitch);
  SPDLOG_CRITICAL("compensator yaw : {}", aiming_eulr.yaw);
  aiming_eulr.pitch = aiming_eulr.pitch + euler.pitch;
  aiming_eulr.yaw = -aiming_eulr.yaw + euler.yaw;

  SPDLOG_INFO("final pitch : {}", aiming_eulr.pitch);
  SPDLOG_CRITICAL("final yaw : {}", aiming_eulr.yaw);
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

  if (method == game::AimMethod::kARMOR) {
    double target_y = distance_ * tan(armor.GetAimEuler().pitch) * 0.001;
    component::Euler aiming_eulr = armor.GetAimEuler();
    int k0 = 1;
    if (ballet_speed == 15) {
      if (distance_ > 1.5 && distance_ < 5) {
        k0 = 1.3;
      } else {
        k0 = 1.6;
      }

      aiming_eulr.pitch =
          PitchTrajectoryCompensation(distance_, -target_y, ballet_speed) * 1.3;
    } else if (ballet_speed == 18) {
      if (distance_ > 1.5 && distance_ < 5) {
        k0 = 1.3;
      } else {
        k0 = 1.6;
      }
      aiming_eulr.pitch =
          PitchTrajectoryCompensation(distance_, -target_y, ballet_speed) * 1.3;
    } else {
      if (distance_ > 1.5 && distance_ < 5) {
        k0 = 1.3;
      } else {
        k0 = 1.6;
      }
      double target_y = distance_ * tan(armor.GetAimEuler().pitch) * 0.001;
      component::Euler aiming_eulr = armor.GetAimEuler();
      aiming_eulr.pitch =
          PitchTrajectoryCompensation(distance_, -target_y, ballet_speed) * 1.3;
    }
    armor.SetAimEuler(aiming_eulr);
  }
}
double Compensator::MonoDirectionalAirResistanceModel(double s, double v,
                                                      double angle) {
  double z;  // ROS坐标系下
  // t为给定v与angle时的飞行时间
  double k = 0.038;
  double t = ((exp(k * s) - 1) / (k * v * cos(angle)));
  // z为给定v与angle时的高度
  z = (v * sin(angle) * t - kG * t * t / 2);
  // printf("model %f %f\n", t, z);
  return z;
}
double Compensator::PitchTrajectoryCompensation(double s, double z, double v) {
  float z_temp, z_actual, dz;  // ROS坐标系下
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++) {
    angle_pitch = atan2(z_temp, s);  // rad
    z_actual = MonoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3 * (z - z_actual);
    z_temp = z_temp + dz;
    SPDLOG_ERROR(
        "iteration num {}: angle_pitch {}, temp target z:{}, err of "
        "z:{}, "
        "s:{}",
        i + 1, angle_pitch * 180 / M_PI, z_temp, dz, s);
    if (fabsf(dz) < 0.00001) {
      break;
    }
  }
  return angle_pitch;
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
