#include "ekf.hpp"

#include <cmath>
#include <vector>

#include "spdlog/spdlog.h"

std::vector<ceres::Jet<double, 5>> LinerModel(
    const std::vector<ceres::Jet<double, 5>> x0, double delta_t) {
  std::vector<ceres::Jet<double, 5>> x1(5);
  x1[0] = x0[0] + delta_t * x0[1];  // 0.1
  x1[1] = x0[1];                    // 100
  x1[2] = x0[2] + delta_t * x0[3];  // 0.1
  x1[3] = x0[3];                    // 100
  x1[4] = x0[4];                    // 0.01
  SPDLOG_DEBUG("LinerModel");
  return x1;
}

std::vector<ceres::Jet<double, 5>> ConvertFromCoordToEuler(
    const std::vector<ceres::Jet<double, 5>> coord) {
  std::vector<ceres::Jet<double, 5>> result(3);
  // pitch
  result[0] = ceres::atan2(
      coord[4], ceres::sqrt(coord[0] * coord[0] + coord[2] * coord[2]));
  // yaw
  result[1] = ceres::atan2(coord[2], coord[0]);
  // distance
  result[2] = ceres::sqrt(coord[0] * coord[0] + coord[2] * coord[2] +
                          coord[4] * coord[4]);
  return result;
}

void EKF::InnerInit(const Matx51d& Xe) {
  cv::cv2eigen(Xe, EXe);
  EP = EMatx55d::Identity();
  EQ = EMatx55d::Identity();
  ER = EMatx33d::Identity();
}

EKF::EKF(const Matx51d& Xe) { InnerInit(Xe); }

EKF::~EKF() { SPDLOG_TRACE("Destruted."); }

void EKF::Init(const std::vector<double>& vec) {
  if (method_ == component::FilterMethod::kUNKNOWN)
    method_ = component::FilterMethod::kEKF;
  InnerInit(Matx51d(vec[0], vec[1], vec[2], vec[3], vec[4]));
}

const cv::Mat& EKF::Predict(const cv::Mat& measurements) {
  std::vector<ceres::Jet<double, 5>> EXe_auto_jet(5), EXp_auto_jet(5);

  for (int i = 0; i < 5; i++) {
    EXe_auto_jet[i].a = EXe[i];
    EXe_auto_jet[i].v[i] = 1;
  }
  EXp_auto_jet = LinerModel(EXe_auto_jet, measurements.at<double>(0));
  for (int i = 0; i < 5; i++) {
    EXp[i] = EXp_auto_jet[i].a;
    EF.block(i, 0, 1, 5) = EXp_auto_jet[i].v.transpose();
  }

  EP = EF * EP * EF.transpose() + EQ;
  cv::eigen2cv(EXp, Xp);
  SPDLOG_DEBUG("Predicted");

  return Xp;
}

const cv::Mat& EKF::Update(const cv::Mat& measurements) {
  std::vector<ceres::Jet<double, 5>> EXp_auto_jet(5), EYp_auto_jet(3);

  EMatx31d Y;
  Matx31d m(measurements.at<double>(0), measurements.at<double>(2),
            measurements.at<double>(4));
  cv::cv2eigen(m, Y);

  for (int i = 0; i < 5; i++) {
    EXp_auto_jet[i].a = EXp[i];
    EXp_auto_jet[i].v[i] = 1;
  }
  EYp_auto_jet = ConvertFromCoordToEuler(EXp_auto_jet);
  for (int i = 0; i < 3; i++) {
    EYp[i] = EYp_auto_jet[i].a;
    EH.block(i, 0, 1, 5) = EYp_auto_jet[i].v.transpose();
  }
  EK = EP * EH.transpose() * (EH * EP * EH.transpose() + ER).inverse();
  EXe = EXp + EK * (Y - EYp);
  EP = (EMatx55d::Identity() - EK * EH) * EP;
  cv::eigen2cv(EXe, Xe);

  SPDLOG_DEBUG("Updated");
  return Xe;
}
