#include "ekf.hpp"

#include <cmath>

#include "spdlog/spdlog.h"

void EKF::InnerInit(const Vec5d& Xe) {
  this->Xe = Xe;
  cv::setIdentity(P);
  cv::setIdentity(Q);
  cv::setIdentity(R);
}
EKF::EKF() { SPDLOG_TRACE("Constructed."); }

EKF::EKF(const Vec5d& Xe = Vec5d::zeros()) { InnerInit(Xe); }

EKF::~EKF() { SPDLOG_TRACE("Destruted."); }

void EKF::Init(const std::vector<double>& vec) {
  InnerInit(Vec5d(vec[0], vec[1], vec[2], vec[3], vec[4]));
}

const cv::Mat& EKF::Predict(const cv::Mat& measurements, const cv::Mat& frame) {
  (void)frame;
  return measurements;
}
