#pragma once

#include <cmath>

#include "armor.hpp"
#include "common.hpp"
#include "tbb/concurrent_vector.h"
#define BIG_ARMOR_LEN 217.5
#define BIG_ARMOR_WID 49.5
#define SMALL_ARMOR_LEN 137.2
#define SMALL_ARMOR_WID 54.1

// definitions of constants
#define PI 3.1415926535  // circle ratio
#define EE 2.7182818284  // nature exponent
#define KK 0.01          // air attrition coefficiency
#define GG \
  9.7883  // g of ShenZhen,g of ChangSha is:9.7915,calculated using
          // g=9.7803(1+0.0053024sin虏蠄-0.000005sin虏2蠄) m/s虏
#define CAMERA_BIAS_Y \
  0.01  // 0.0248  //unit:m the vertical distance between optical center of
        // camera and pitch pivot
#define CAMERA_BIAS_X \
  0.01  // 0.19342 //unit:m the horizontal distance between optical center of
        // camera and pitch pivot
#define GUN_BIAS_Y \
  0.02621  // unit:m the vertical distance between axis of gun center and pitch
           // pivot
#define GUN_BIAS_X \
  0.19342  // unit:m the horizontal distance between axis of gun center and
           // pitch pivot
           //---!!!temporally,GUN_BIAS_X = CAMERA_BIAS_X!!!---
#define FIX 10
class Compensator {
 private:
  double distance_;
  cv::Mat cam_mat_, distor_coff_;
  double gun_cam_distance_; /* 枪口到镜头的距离 */
  game::Arm arm_;

  void SolveAngles(Armor& armor, const component::Euler& euler);
  void CompensateGravity(Armor& armor, const double ballet_speed,
                         game::AimMethod method);

  void VisualizePnp(Armor& armor, const cv::Mat& output, bool add_lable);

  void UpdateImgPoints(std::vector<cv::Point2f>& img, double k,
                       std::vector<cv::Point2f>& img_out);

 public:
  Compensator();
  Compensator(const std::string& cam_mat_path,
              const game::Arm& arm = game::Arm::kINFANTRY);
  ~Compensator();

  void SetArm(const game::Arm& arm);
  void LoadCameraMat(const std::string& path);

  void PnpEstimate(Armor& armor);
  void Apply(tbb::concurrent_vector<Armor>& armors, const double ballet_speed,
             const component::Euler& euler, game::AimMethod method);

  void Apply(Armor& armor, const double ballet_speed,
             const component::Euler& euler, game::AimMethod method);

  void VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                       const cv::Mat& output, int verbose = 1);
  void UpdateImgPoints(std::vector<cv::Point2f>& img, double k,
                       std::vector<cv::Point2f>& img_out);

  double GetRecompensation(double camera_pitch, double camera_distance,
                           double gimbal_pitch, double bullet_speed);
  void ShengJin(double a, double b, double c, double d,
                std::vector<double>& X123);
#ifdef RMU2021
  double SolveSurfaceLanchAngle(cv::Point2f target, double ballet_speed);
  cv::Vec3f EstimateWorldCoord(Armor& armor);
#endif
};
