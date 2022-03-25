#pragma once

#include "armor.hpp"
#include "common.hpp"
#include "tbb/concurrent_vector.h"

class Compensator {
 private:
  double ballet_speed_, distance_;
  cv::Mat cam_mat_, distor_coff_;
  double gun_cam_distance_;  //枪口到镜头的距离

  void SolveAngles(Armor& armor);
  void CompensateGravity(Armor& armor, component::Euler euler);

  void VisualizePnp(Armor& armor, const cv::Mat& output, bool add_lable);

 public:
  Compensator();
  Compensator(const std::string& cam_mat_path,
              const game::Arm& arm = game::Arm::kINFANTRY);
  ~Compensator();

  void SetArm(const game::Arm& arm);
  void LoadCameraMat(const std::string& path);

  void PnpEstimate(Armor& armor);
  void Apply(tbb::concurrent_vector<Armor>& armors, const cv::Mat& frame,
             const component::Euler& euler);

  void VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                       const cv::Mat& output, int verbose = 1);

#ifdef RMU2021
  double SolveSurfaceLanchAngle(cv::Point2f target);
  cv::Vec3f EstimateWorldCoord(Armor& armor);
#endif
};
