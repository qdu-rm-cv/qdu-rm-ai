#include "filter.hpp"

#include <cmath>

#include "spdlog/spdlog.h"

void Kalman::Init(int states, int measurements, int inputs) {
  kalman_filter_.init(states, measurements, inputs, CV_64F);
}

Kalman::Kalman() { SPDLOG_TRACE("Constructed."); }

Kalman::Kalman(int states, int measurements, int inputs) {
  Init(states, measurements, inputs);
  SPDLOG_TRACE("Constructed.");
}

Kalman::~Kalman() { SPDLOG_TRACE("Destructed."); }

const cv::Mat& Kalman::Predict() { return kalman_filter_.predict(); }

const cv::Mat& Kalman::Update(cv::Mat& measurements) {
  return kalman_filter_.correct(measurements);
}

void EKF::Init(const Vec5d& Xe) {
  this->Xe = Xe;
  cv::setIdentity(P);
  cv::setIdentity(Q);
  cv::setIdentity(R);
}

EKF::EKF() { SPDLOG_TRACE("Constructed."); }

EKF::EKF(const Vec5d& Xe = Vec5d::zeros()) { Init(Xe); }

EKF::~EKF() { SPDLOG_TRACE("Destruted."); }

const cv::Mat& EKF::Predict() {}

const cv::Mat& EKF::Update() {}