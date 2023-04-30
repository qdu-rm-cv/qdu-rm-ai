#include "video_server.hpp"

template <>
int Context_<int>::HandleMessage(const std::string& msg) {
  SPDLOG_INFO("message size:{} // {}", msg.size(), msg);
  // Format Recved 'msg' to the type you want;
  return msg.size();
}

VideoServer::VideoServer() {
  service_.onmessage = [&](const WebSocketChannelPtr& channel,
                           const std::string& msg) {
    auto ctx = channel->getContextPtr<VideoContext>();
    auto data = ctx->Encode(data_);
    ctx->HandleMessage(msg);
    ctx->send(data);
  };
  server_thread = std::thread(&VideoServer::Run, this);
}

VideoServer::~VideoServer() { server_thread.join(); }

std::string VideoContext::Encode(const cv::Mat& frame) {
  std::vector<uchar> buf;
  std::vector<int> params;
  params.resize(3, 0);
  params[0] = cv::IMWRITE_JPEG_QUALITY;
  params[1] = 90;
  cv::imencode(".jpg", frame, buf, params);
  std::string msg(buf.begin(), buf.end());
  SPDLOG_WARN("Send Over");
  pre_meg_ = msg;
  return msg;
}
