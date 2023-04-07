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
  double k = 1.23;
  cv::Point2f t1 = (armor.ImageVertices()[1] - armor.ImageVertices()[0]) * k;
  // 向量t1,t2
  cv::Point2f t2 = (armor.ImageVertices()[2] - armor.ImageVertices()[3]) * k;
  cv::Point2f tr = armor.ImageVertices()[0];  // Right bottom point
  cv::Point2f tl = armor.ImageVertices()[3];  // Left bottom point

  img.clear();
  img.push_back(tr);
  img.push_back(tr + t1);
  img.push_back(tl + t2);
  img.push_back(tl);
  double k2;  // k2值是目标装甲板的长宽比
  if (armor.IsBigArmor()) {
    k2 = 230 / 127 * cos(15 / 180 * M_PI);
    SPDLOG_CRITICAL("BIG!!!");
  } else if (armor.GetModel() == game::Model::kBUFF) {
    k2 = 1;  // TODO(gui) : 等Buff和buff_detector完成后修改
    SPDLOG_ERROR("Error param of buff has not been set!");
    return;
  } else {
    k2 = 135 / 125 * cos(15 / 180 * M_PI);
    SPDLOG_WARN("SMALL!!!");
  }

  UpdateImgPoints(img, k2, img_out);
  double k3 = 125 / cv::norm(img_out[0] - img_out[1]);

  cv::Point2f new_img_center = cv::Point2f(
      (img_out[0].x + img_out[1].x + img_out[2].x + img_out[3].x) / 4,
      (img_out[0].y + img_out[1].y + img_out[2].y + img_out[3].y) / 4);
  double center_diff_x = abs(armor.ImageCenter().x - new_img_center.x) *
                         k3;  // 重构之后装甲板的中心会有偏移
  double center_diff_y = abs(armor.ImageCenter().y - new_img_center.y) * k3;

  cv::solvePnP(armor.PhysicVertices(), armor.ImageVertices() /* img_out */,
               cam_mat_, distor_coff_, rot_vec, trans_vec, false,
               cv::SOLVEPNP_ITERATIVE);
  /* trans_vec.at<double>(0, 0) -= center_diff_x;
  trans_vec.at<double>(1, 0) -= center_diff_y;
 */
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
  SPDLOG_INFO("compensator yaw : {}", aiming_eulr.yaw);
  aiming_eulr.pitch = aiming_eulr.pitch + euler.pitch;
  aiming_eulr.yaw = -aiming_eulr.yaw + euler.yaw;

  SPDLOG_INFO("final pitch : {}", aiming_eulr.pitch);
  SPDLOG_INFO("final yaw : {}", aiming_eulr.yaw);
  armor.SetAimEuler(aiming_eulr);
}

void Compensator::Apply(tbb::concurrent_vector<Armor>& armors,
                        const cv::Mat& frame, const double ballet_speed,
                        const component::Euler& euler, game::AimMethod method) {
  cv::Point2f frame_center(frame.cols / 2, frame.rows / 2);
  std::sort(armors.begin(), armors.end(),
            [frame_center](Armor& armor1, Armor& armor2) {
              if (armor1.GetArea() > armor2.GetArea()) {
                return true;
              } else {
                if (cv::norm(armor1.image_center_ - frame_center) >
                    cv::norm(armor2.image_center_ - frame_center))
                  return false;
                return true;
                // SPDLOG_CRITICAL("11111111111111");
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
  // TODO： test
  // component::Euler t = armor.GetAimEuler();
  // double new_pitch =
  //     GetRecompensation(t.pitch, distance_, euler.pitch, ballet_speed);
  // t.pitch = new_pitch;
  // t.yaw += 0;
  // armor.SetAimEuler(t);
}
double Compensator::GetRecompensation(double camera_pitch,
                                      double camera_distance,
                                      double gimbal_pitch,
                                      double bullet_speed) {
  double pg_angle = atan(GUN_BIAS_Y / GUN_BIAS_X);
  double pg_distance = sqrt(GUN_BIAS_X * GUN_BIAS_X + GUN_BIAS_Y * GUN_BIAS_Y);
  // transform these to standard unit first
  camera_pitch = camera_pitch / 180 * PI;
  gimbal_pitch = gimbal_pitch / 180 * PI;
  camera_distance /= 1000;

  // calculate pitch angle of pivot axis
  double pivot_horizontal_distance = camera_distance * cos(camera_pitch) -
                                     CAMERA_BIAS_Y * sin(gimbal_pitch) +
                                     CAMERA_BIAS_X * cos(gimbal_pitch);
  double pivot_vertical_distance = camera_distance * sin(camera_pitch) +
                                   CAMERA_BIAS_Y * cos(gimbal_pitch) +
                                   CAMERA_BIAS_X * sin(gimbal_pitch);
  double pivot_pitch_angle =
      atan(pivot_vertical_distance / pivot_horizontal_distance);

  double pivot_distance = pivot_horizontal_distance / cos(pivot_pitch_angle);

  double gun_horizontal_distance =
      pivot_horizontal_distance -
      pg_distance * cos(pivot_pitch_angle + pg_angle);
  double gun_vertical_distance =
      pivot_vertical_distance - pg_distance * sin(pivot_pitch_angle + pg_angle);
  double target_angle = atan(gun_vertical_distance / gun_horizontal_distance);
  // double gun_distance = pivot_distance - pg_distance * cos(pg_angle);
  // initialization and declaration of iteration
  double temp_angle = target_angle;
  double target_height = gun_vertical_distance;
  // cout<<temp_angle*180/PI<<endl;
  double temp_height;
  double vy0;
  double vx0;
  double t;
  double real_height;
  double delta_height;

  // fixed iteration times
  for (int i = 0; i < FIX; i++) {
    // updata parameters
    gun_horizontal_distance =
        pivot_horizontal_distance - pg_distance * cos(temp_angle + pg_angle);
    temp_height = gun_horizontal_distance * tan(temp_angle);
    vy0 = sin(temp_angle) * bullet_speed;
    vx0 = cos(temp_angle) * bullet_speed;

    t = (exp(KK * gun_horizontal_distance) - 1) / (KK * vx0);
    real_height = vy0 * t - 0.5 * GG * t * t;
    delta_height = target_height - real_height;

    // angle should be updated as soon as we worked out delta
    temp_angle = atan((temp_height + delta_height) / gun_horizontal_distance);
    //            printf("height:%f  ", real_height);
    //            printf("delta:%f  ", delta_height);
  }
  return temp_angle * 180 / PI;
}

void Compensator::VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                                  const cv::Mat& output, int verbose) {
  for (auto& armor : armors) {
    VisualizePnp(armor, output, verbose > 1);
  }
}
#if 1
/*下下策，每10cm为一个间隔段硬补*/
void Compensator::CompensateGravity(Armor& armor, const double ballet_speed,
                                    game::AimMethod method) {
  component::Euler aiming_eulr = armor.GetAimEuler();
  SPDLOG_ERROR("start pitch{}", aiming_eulr.pitch);
  SPDLOG_ERROR("le");
  distance_ = 4.0;
  if (distance_ < 3) {
  } else if (distance_ >= 3.0 && distance_ < 3.2) {
    aiming_eulr.pitch += 2.2 / 180 * CV_PI;
  } else if (distance_ >= 3.2 && distance_ < 3.4) {
    aiming_eulr.pitch += 2.8 / 180 * CV_PI;
  } else if (distance_ >= 3.4 && distance_ < 3.6) {
    aiming_eulr.pitch += 3.13 / 180 * CV_PI;
  } else if (distance_ >= 3.6 && distance_ < 3.8) {
    aiming_eulr.pitch += 3.16 / 180 * CV_PI;
  } else if (distance_ >= 3.8 && distance_ < 4.0) {
    aiming_eulr.pitch += 3.19 / 180 * CV_PI;
  } else if (distance_ >= 4.0 && distance_ < 4.2) {
    aiming_eulr.pitch += 3.23 / 180 * CV_PI;
  } else if (distance_ >= 4.4 && distance_ < 4.6) {
    aiming_eulr.pitch += 3.25 / 180 * CV_PI;
  } else if (distance_ >= 4.6 && distance_ < 4.8) {
    aiming_eulr.pitch += 3.27 / 180 * CV_PI;
  } else if (distance_ >= 4.8 && distance_ < 5.0) {
    aiming_eulr.pitch += 3.29 / 180 * CV_PI;
  } else if (distance_ >= 5.2 && distance_ < 5.4) {
    aiming_eulr.pitch += 3.3 / 180 * CV_PI;
  } else if (distance_ >= 5.4 && distance_ < 5.6) {
    aiming_eulr.pitch += 3.5 / 180 * CV_PI;
  } else {
  }
  // aiming_eulr.yaw -= 1.0 / 180 * CV_PI;
  // SPDLOG_INFO("diff is {}", pitch - aiming_eulr.pitch);
  // SPDLOG_INFO("{} <=> {}", aiming_eulr.pitch, pitch);
  // // SPDLOG_INFO("Distance : {} <=> Now pitch : {}", distance_, pitch);
  // aiming_eulr.pitch = pitch;
  //}
  armor.SetAimEuler(aiming_eulr);
  SPDLOG_ERROR("end pitch{}", aiming_eulr.pitch);
  SPDLOG_DEBUG("Armor Euler is setted");
}
#else
void Compensator::CompensateGravity(Armor& armor, const double ballet_speed,
                                    game::AimMethod method) {
  component::Euler aiming_eulr = armor.GetAimEuler();
  if ((aiming_eulr.pitch < 0.0 || aiming_eulr.pitch > 3) && distance_ < 3.0 &&
      method == game::AimMethod::kARMOR) {
    // ping pao
    double pitch = aiming_eulr.pitch;
    SPDLOG_ERROR("pitch {}", pitch);
    double A = (distance_ * kG) / (ballet_speed * ballet_speed);
    double B = tan(pitch) / cos(pitch);
    /* B = sin(pitch) / (cos(pitch) * cos(pitch)) */
    double C = 1 / cos(pitch);
    double D = B * B + C * C;
    double E = 2 * B * (A + B);
    double F = (A + B) * (A + B) - C * C;

    double temporary_result =
        0.5 * acos((E + pow(-1, 0) * sqrt(E * E - 4 * D * F)) / (2 * D));
    /* 当判断时枪口会斗 */
    pitch = temporary_result * 0.9;
    aiming_eulr.yaw -= 0.5 / 180 * CV_PI;
    SPDLOG_CRITICAL("diff is {}", pitch - aiming_eulr.pitch);
    SPDLOG_INFO("PingPao Distance : {} <=> New pitch : {}", distance_, pitch);
    aiming_eulr.pitch = pitch;
  } else if ((aiming_eulr.pitch < 0.0 || aiming_eulr.pitch > 3) &&
             distance_ >= 3.0 && method == game::AimMethod::kARMOR) {
    double A = tan(aiming_eulr.pitch);
    double B =
        kG * distance_ * cos(aiming_eulr.pitch) / (2 * pow(ballet_speed, 2));
    double a = 1;
    double b = A * A;
    double c = 2 * A * B;
    double d = B * B - 1;
    std::vector<double> X123;
    std::vector<double> X_f;
    ShengJin(a, b, c, d, X123);
    // selsct
    for (int i = 0; i < X123.size(); i++) {
      if (0 < X123[i] &&
          X123[i] < cos(10 / 180 * M_PI) * cos(10 / 180 * M_PI)) {
        X_f.push_back(X123[i]);
      }
    }
    std::vector<double> pitch_f;
    for (int i = 0; i < X_f.size(); i++) {
      double cos_b = sqrt(X_f[i]);
      double bt = acos(cos_b);
      pitch_f.push_back(bt);
    }
    if (pitch_f.empty()) {
      SPDLOG_WARN("No result!");
      return;
    }
    std::sort(pitch_f.begin(), pitch_f.end(),
              [](double& a, double& b) { return a > b; });
    double result = pitch_f.front();
    aiming_eulr.pitch = result;
    SPDLOG_INFO("XiePao Distance : {} <=> Now pitch : {}", distance_,
                aiming_eulr.pitch);
  } else {
    double pitch = aiming_eulr.pitch;
    double A = distance_ / sin(pitch);
    double B = 1 / (2 * kG);
    double result1 = (-1 + sqrt(1 - 4 * A * B)) / 2 * B;
    // double result2 = ;
    double final_result = asin(result1);
    pitch = final_result;
    if (distance_ > 3 && distance_ <= 5) {
      pitch *= 0.9;
      aiming_eulr.yaw -= 0.1 / 180 * CV_PI;
    }
    if (distance_ > 7) {
      pitch *= 0.85;
      aiming_eulr.yaw -= 0.3 / 180 * CV_PI;
    }
    SPDLOG_INFO("XiePao Buff Distance : {} <=> Now pitch : {}", distance_,
                pitch);
  }
  armor.SetAimEuler(aiming_eulr);
  SPDLOG_DEBUG("Armor Euler is setted");
}
void Compensator::ShengJin(double a, double b, double c, double d,
                           std::vector<double>& X123) {
  /************************************************************************/
  /* 盛金公式求解三次方程的解
     德尔塔f=B^2-4AC
     这里只要了实根，虚根需要自己再整理下拿出来
  */
  /************************************************************************/
  double A = b * b - 3 * a * c;
  double B = b * c - 9 * a * d;
  double C = c * c - 3 * b * d;
  double f = B * B - 4 * A * C;
  double i_value;
  double Y1, Y2;
  if (fabs(A) < 1e-6 && fabs(B) < 1e-6)  // 公式1
  {
    X123.push_back(-b / (3 * a));
    X123.push_back(-b / (3 * a));
    X123.push_back(-b / (3 * a));
  } else if (fabs(f) < 1e-6)  // 公式3
  {
    double K = B / A;
    X123.push_back(-b / a + K);
    X123.push_back(-K / 2);
    X123.push_back(-K / 2);
  } else if (f > 1e-6)  // 公式2
  {
    Y1 = A * b + 3 * a * (-B + sqrt(f)) / 2;
    Y2 = A * b + 3 * a * (-B - sqrt(f)) / 2;
    double Y1_value = (Y1 / fabs(Y1)) * pow((double)fabs(Y1), 1.0 / 3);
    double Y2_value = (Y2 / fabs(Y2)) * pow((double)fabs(Y2), 1.0 / 3);
    X123.push_back((-b - Y1_value - Y2_value) / (3 * a));  // 虚根我不要
    // 虚根还是看看吧，如果虚根的i小于0.1，则判定为方程的一根吧。。。
    i_value = sqrt(3.0) / 2 * (Y1_value - Y2_value) / (3 * a);
    if (fabs(i_value) < 1e-1) {
      X123.push_back((-b + 0.5 * (Y1_value + Y2_value)) / (3 * a));
    }
  } else if (f < -1e-6)  // 公式4
  {
    double T = (2 * A * b - 3 * a * B) / (2 * A * sqrt(A));
    double S = acos(T);
    X123.push_back((-b - 2 * sqrt(A) * cos(S / 3)) / (3 * a));
    X123.push_back((-b + sqrt(A) * (cos(S / 3) + sqrt(3.0) * sin(S / 3))) /
                   (3 * a));
    X123.push_back((-b + sqrt(A) * (cos(S / 3) - sqrt(3.0) * sin(S / 3))) /
                   (3 * a));
  }
}
#endif
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
