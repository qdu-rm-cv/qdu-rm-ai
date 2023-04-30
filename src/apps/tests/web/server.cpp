#include "gtest/gtest.h"
#include "log.hpp"
#include "opencv2/opencv.hpp"
#include "video_server.hpp"

TEST(TestWeb, TestCamServer) {
  component::logger::SetLogger();
  cv::VideoCapture cap(kPATH_IMAGE + "../other/vtest.avi");
  int ct = 0;
  bool grabing = true;
  cv::Mat frame;

  VideoServer server;
  server.Open("9999");
  server.SetInterval(30);

  if (cap.isOpened()) {
    int cts = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));

    while (grabing && server.prepared_) {
      cap >> frame;
      grabing = !frame.empty();
      if (grabing) {
        ct += 1;
        cv::imshow("fff", frame);
        // server.Encode(frame);
        server.data_ = frame;
        if (ct % 10 == 0) {
          SPDLOG_WARN("frame_count : {}", ct);
        }
      }
      if (ct == cts) {
        ct = 0;
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
      }
      cv::waitKey(fps);
    }
  }
}
