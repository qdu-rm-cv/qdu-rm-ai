#include "compensator.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kG = 9.80665;

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

Compensator::Compensator(const std::string& cam_mat_path) {
  SPDLOG_TRACE("Constructed.");
  // TODO : 和机械、兵种相关，后期放到namespace
  gun_cam_distance_ = 0;
  LoadCameraMat(cam_mat_path);
}

Compensator::~Compensator() { SPDLOG_TRACE("Destructed."); }

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

void Compensator::SolveAngles(Armor& armor, component::Euler euler) {
  (void)euler;
  component::Euler aiming_eulr;
  cv::Mat rot_vec, trans_vec;

  cv::solvePnP(armor.PhysicVertices(), armor.ImageVertices(), cam_mat_,
               distor_coff_, rot_vec, trans_vec, false, cv::SOLVEPNP_ITERATIVE);

  trans_vec.at<double>(1, 0) -= gun_cam_distance_;
  armor.SetRotVec(rot_vec), armor.SetTransVec(trans_vec);

  double x_pos = armor.GetTransVec().at<double>(0, 0);
  double y_pos = armor.GetTransVec().at<double>(1, 0);
  double z_pos = armor.GetTransVec().at<double>(2, 0);
  SPDLOG_WARN("x : {}, y : {}, z : {} ", x_pos, y_pos, z_pos);
  distance_ = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);

  if (distance_ > 5000) {
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
  armor.SetAimEuler(aiming_eulr);
}

void Compensator::Apply(tbb::concurrent_vector<Armor>& armors,
                        const cv::Mat& frame, const component::Euler& euler) {
  for (auto& armor : armors) {
    if (armor.GetModel() == game::Model::kUNKNOWN) {
      armor.SetModel(game::Model::kINFANTRY);
      SPDLOG_ERROR("Hasn't set model.");
    }
    SolveAngles(armor, euler);
    // CompensateGravity(armor, euler);
  }
  cv::Point2f frame_center(frame.cols / 2, frame.rows / 2);
  std::sort(armors.begin(), armors.end(),
            [frame_center](Armor& armor1, Armor& armor2) {
              return cv::norm(armor1.ImageCenter() - frame_center) <
                     cv::norm(armor2.ImageCenter() - frame_center);
            });
}

void Compensator::VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                                  const cv::Mat& output, int verbose) {
  for (auto& armor : armors) {
    VisualizePnp(armor, output, verbose > 1);
  }
}

void Compensator::CompensateGravity(Armor& armor, component::Euler euler) {
  component::Euler aiming_eulr;
  aiming_eulr.pitch = armor.GetAimEuler().pitch + euler.pitch;
  float compensate_pitch = atan(
      (sin(aiming_eulr.pitch) - 0.5 * kG * distance_ / pow(ballet_speed_, 2)) /
      cos(aiming_eulr.pitch));
  aiming_eulr.pitch = (armor.GetAimEuler().pitch + compensate_pitch) * (-0.01);
  armor.SetAimEuler(aiming_eulr);

  // 人为补偿
  // aiming_eulr.pitch += 3.2 / 180.0 * CV_PI;
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
double Compensator::SolveSurfaceLanchAngle(cv::Point2f target) {
  const double v_2 = pow(ballet_speed_, 2);
  const double up_base =
      std::sqrt(std::pow(ballet_speed_, 4) -
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