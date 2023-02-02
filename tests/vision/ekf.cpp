#include "ekf.hpp"

#include "buff_detector.hpp"
#include "gtest/gtest.h"
#include "log.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const std::vector<cv::Point2d> points = {
    cv::Point2d(50., 50.),  cv::Point2d(100., 50.),  cv::Point2d(150., 50.),
    cv::Point2d(200., 50.), cv::Point2d(250., 50.),  cv::Point2d(300., 50.),
    cv::Point2d(350., 50.), cv::Point2d(400., 50.),  cv::Point2d(450., 50.),
    cv::Point2d(500., 50.), cv::Point2d(550., 50.),  cv::Point2d(600., 50.),
    cv::Point2d(650., 50.), cv::Point2d(700., 50.),  cv::Point2d(750., 50.),
    cv::Point2d(800., 50.), cv::Point2d(850., 50.),  cv::Point2d(900., 50.),
    cv::Point2d(950., 50.), cv::Point2d(1000., 50.), cv::Point2d(1050., 50.)};

const std::string kVIDEO = "../../../../redbuff01.avi";
const std::string kPARAM = "../../../runtime/RMUT2021_Buff.json";

}  // namespace

TEST(TestVision, TestEKF) {
  component::logger::SetLogger();
  EKF filter;
  SPDLOG_WARN("Filter");
  cv::Mat predict_mat;
  cv::Mat img(300, 1200, CV_8UC3, cv::Scalar(0, 0, 0));

  for (auto& pt : points) {
    cv::Mat point_mat(pt);
    predict_mat = filter.Predict(point_mat);
    SPDLOG_WARN("Predicted");
    cv::Point2d predict_pt(predict_mat.at<double>(0, 0),
                           predict_mat.at<double>(0, 1));
    filter.Update(predict_mat);
    cv::circle(img, pt, 3, draw::kBLUE, 2);
    cv::circle(img, predict_pt, 3, draw::kGREEN, 2);

    // cv::imshow("img", img);
    // cv::waitKey(0);
  }
  cv::destroyWindow("img");
}
