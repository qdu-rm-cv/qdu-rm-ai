#include "compensator.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kG = 9.7945;
const double kINFANTRY = 60;
const double kHERO = 10.;
const double kSENTRY = 10.;

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
  std::vector<cv::Point2f> img;         // Points of 2D after adjusted
  std::vector<cv::Point2f> img_out(4);  // Points of 2D after update
  /*调整识别到的像素坐标,手动消除与处理带来的2D坐标
  不准的为问题,该参数可以根据ui_param灯条的变形情况
  来确定*/
  double k = 1.15;
  cv::Point2f t1 =
      (armor.ImageVertices()[1] - armor.ImageVertices()[0]) * k;  //向量t1,t2
  cv::Point2f t2 = (armor.ImageVertices()[2] - armor.ImageVertices()[3]) * k;
  cv::Point2f tr = armor.ImageVertices()[0];  // Right bottom point
  cv::Point2f tl = armor.ImageVertices()[3];  // Left bottom point

  img.clear();
  img.push_back(tr);
  img.push_back(tr + t1);
  img.push_back(tl + t2);
  img.push_back(tl);
  double k2;  // k2值是目标装甲板的长宽比
  if (game::HasBigArmor(armor.GetModel())) {
    k2 = 230 / 127 * cos(15 / 180 * M_PI);
  } else if (armor.GetModel() == game::Model::kBUFF) {
    k2 = 1;  // TODO:等Buff和buff_detector完成后修改
    SPDLOG_ERROR("Error param of buff has not been set!");
    return;
  } else {
    k2 == 135 / 125 * cos(15 / 180 * M_PI);
  }

  UpdateImgPoints(img, k2, img_out);

  cv::solvePnP(armor.PhysicVertices(), /* armor.ImageVertices() */ img_out,
               cam_mat_, distor_coff_, rot_vec, trans_vec, false,
               cv::SOLVEPNP_ITERATIVE);
  // TODO:TO DELETE THE "//" BELOW
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
                        const cv::Mat& frame, const double ballet_speed,
                        const component::Euler& euler, game::AimMethod method) {
  cv::Point2f frame_center(frame.cols / 2, frame.rows / 2);
  std::sort(armors.begin(), armors.end(),
            [frame_center](Armor& armor1, Armor& armor2) {
              return cv::norm(armor1.ImageCenter() - frame_center) <
                     cv::norm(armor2.ImageCenter() - frame_center);
            });
  auto& armor = armors.front();
  if (armor.GetModel() == game::Model::kUNKNOWN) {
    armor.SetModel(game::Model::kINFANTRY);
    SPDLOG_ERROR("Hasn't set model.");
  }
  SolveAngles(armor, euler);
  CompensateGravity(armor, ballet_speed, method);
}

void Compensator::Apply(Armor& armor, const cv::Mat& frame,
                        const double ballet_speed,
                        const component::Euler& euler, game::AimMethod method) {
  cv::Point2f frame_center(frame.cols / 2, frame.rows / 2);

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
  component::Euler aiming_eulr = armor.GetAimEuler();
  if (method == game::AimMethod::kARMOR) {
    double pitch = -aiming_eulr.pitch;
    double A = (distance_ * kG) / (ballet_speed * ballet_speed);
    double B = tan(pitch) / cos(pitch);
    /* B = sin(pitch) / (cos(pitch) * cos(pitch)) */
    double C = 1 / cos(pitch);
    double D = B * B + C * C;
    double E = 2 * B * (A + B);
    double F = (A + B) * (A + B) - C * C;

    double temporary_result =
        0.5 * acos((-E + pow(-1, 0) * sqrt(E * E - 4 * D * F)) / (2 * D));
    if (temporary_result > pitch) {
      pitch = temporary_result;
    }
    if (distance_ > 3 && distance_ <= 5) {
      pitch *= 0.9;
      aiming_eulr.yaw += 0.1 / 180 * CV_PI;
    }
    if (distance_ > 7) {
      pitch *= 0.85;
      aiming_eulr.yaw += 0.3 / 180 * CV_PI;
    }
    SPDLOG_INFO("Distance : {} <=> Now pitch : {}", distance_, pitch);
    aiming_eulr.pitch = pitch;
  } else if (0) {
    // (void)ballet_speed;
    aiming_eulr.yaw -= 0.3 / 180 * CV_PI;
    double pitch = aiming_eulr.pitch;
    double A = -((distance_ * kG) / (ballet_speed * ballet_speed));
    double B = tan(pitch) / cos(pitch);
    /* B = sin(pitch) / (cos(pitch) * cos(pitch)) */
    double C = 1 / cos(pitch);
    double D = B * B + C * C;
    double E = 2 * B * (A - B);
    double F = (A - B) * (A - B) - C * C;

    for (int i = 0; i < 2; i++) {
      double temporary_result =
          0.5 * acos((E + pow(-1, i) * sqrt(E * E - 4 * D * F)) / (2 * D));
      SPDLOG_DEBUG("temporary_pitch{}", temporary_result);

      if (temporary_result > 0 && temporary_result > pitch &&
          temporary_result < 0.5) {
        pitch = 1.3 * temporary_result;
        SPDLOG_INFO("{} <=> {}", aiming_eulr.pitch, pitch);
        continue;
      }
    }
    aiming_eulr.pitch = pitch;
  } else {
    SPDLOG_WARN("start {}, {}", aiming_eulr.yaw, aiming_eulr.pitch);
    aiming_eulr.yaw += 0.4 / 180 * CV_PI;
    if (aiming_eulr.pitch < 0.15) {
      aiming_eulr.pitch += 1.7 / 180 * CV_PI;
      SPDLOG_WARN("0.1");
    } else if (aiming_eulr.pitch < 0.25) {
      aiming_eulr.pitch += 1.8 / 180 * CV_PI;
      SPDLOG_WARN("0.2");
    } else if (aiming_eulr.pitch < 0.3) {
      aiming_eulr.pitch += 2.5 / 180 * CV_PI;
      SPDLOG_WARN("0.3");
    } else if (aiming_eulr.pitch < 0.4) {
      aiming_eulr.pitch += 2.5 / 180 * CV_PI;
      SPDLOG_WARN("0.4");
    } else if (aiming_eulr.pitch < 0.5) {
      aiming_eulr.pitch += 2.8 / 180 * CV_PI;
      SPDLOG_WARN("0.5");
    } else {
      aiming_eulr.pitch += 3.0 / 180 * CV_PI;
      SPDLOG_WARN("else");
    }
    SPDLOG_WARN(" end {}, {}", aiming_eulr.yaw, aiming_eulr.pitch);
  }
  armor.SetAimEuler(aiming_eulr);
  SPDLOG_DEBUG("Armor Euler is setted");
}
/*图片坐标默认顺序
左下，左上，右上，右下
 */
/*使用前体装甲板底边和书平面平行或者偏差不大并且者相机不能倾斜，
以下函数中提到的length和width均为img中的,1/K为装甲版的实际
长宽比，注意要区分大小装甲版*/
void Compensator::UpdateImgPoints(std::vector<cv::Point2f>& img, double k,
                                  std::vector<cv::Point2f>& img_out) {
  double length, width;
  width = std::max(cv::norm(img[0] - img[1]), cv::norm(img[3] - img[2]));
  length = width * k;
  img_out[0] = img[0];
  img_out[1] = img[0] - cv::Point2f(0, width);
  img_out[2] = img_out[1] + cv::Point2f(length, 0);
  img_out[3] = img_out[2] + cv::Point2f(0, width);
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
