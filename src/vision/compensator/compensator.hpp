#pragma once

#include "armor.hpp"
#include "tbb/concurrent_vector.h"

class Compensator {
 private:
  double ballet_speed_, distance_;
  cv::Mat cam_mat_, distor_coff_;
  double gun_cam_distance_;  //枪口到镜头的距离

  void SolveAngles(Armor& armor, component::Euler euler);
  void CompensateGravity(Armor& armor, component::Euler euler);

  double PinHoleEstimate(Armor& armor);

  void VisualizePnp(Armor& armor, const cv::Mat& output, bool add_lable);

 public:
  Compensator();
  Compensator(const std::string& cam_mat_path);
  ~Compensator();

  void LoadCameraMat(const std::string& path);

  void Apply(tbb::concurrent_vector<Armor>& armors, const cv::Mat& frame,
             const component::Euler& euler);

  void VisualizeResult(tbb::concurrent_vector<Armor>& armors,
                       const cv::Mat& output, int verbose = 1);

#ifdef RM2021
  double SolveSurfaceLanchAngle(cv::Point2f target);
  cv::Vec3f EstimateWorldCoord(Armor& armor);
#endif
};
