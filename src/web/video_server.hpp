#pragma once

#include <thread>

#include "opencv2/opencv.hpp"
#include "server.hpp"

class VideoContext : public Context_<int> {
 private:
  std::string pre_meg_;

 public:
  std::string Encode(const cv::Mat& frame);
};

class VideoServer : public WebServer<cv::Mat, int> {
 private:
  std::thread server_thread;

 public:
  VideoServer();
  ~VideoServer();
};
