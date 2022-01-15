#include <cmath>

#include "ekf.hpp"
#include "spdlog/spdlog.h"

void EKF::Init(const Vec5d& Xe) {
  this->Xe = Xe;
  cv::setIdentity(P);
  cv::setIdentity(Q);
  cv::setIdentity(R);
}

EKF::EKF() { SPDLOG_TRACE("Constructed."); }

EKF::EKF(const Vec5d& Xe = Vec5d::zeros()) { Init(Xe); }

EKF::~EKF() { SPDLOG_TRACE("Destruted."); }

const cv::Mat& EKF::Predict(const cv::Mat& measurements, const cv::Mat& frame) {
  (void)frame;
  return measurements;
}
