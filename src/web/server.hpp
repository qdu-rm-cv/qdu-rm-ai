#pragma once

#include <string>
#include <thread>
#include <vector>

#include "common.hpp"
#include "hv/EventLoop.h"
#include "hv/WebSocketServer.h"
#include "hv/hssl.h"
#include "hv/htime.h"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

template <typename T>
class Context_ : public hv::HttpContext {
 public:
  hv::TimerID timer;

  virtual T HandleMessage(const std::string& msg);
};

template <typename DataType, typename Json, typename Context = Context_<Json>>
class WebServer {
 private:
  hv::WebSocketServer ws_;
  int port_;
  int interval_;

  void Init() {
    interval_ = 0;
    port_ = 0;
    prepared_ = false;
  }

  void OpenPrepare() {
    if (!(interval_ == 0 && port_ == 0)) {
      prepared_ = true;
    } else {
      if (interval_ == 0) interval_ = 100;
      if (port_ == 0) port_ = 9999;
      prepared_ = true;
    }
  }

  void Prepare() {
    hv::HttpService http;
    http.GET("/ping",
             [](const HttpContextPtr& ctx) { return ctx->send("pong"); });

    service_.onopen = [&](const WebSocketChannelPtr& channel,
                          const HttpRequestPtr& req) {
      SPDLOG_INFO("onopen: GET {}\n", req->Path());
      auto ctx = channel->newContextPtr<Context>();

      // send(time) every 1s
      ctx->timer = hv::setInterval(interval_, [channel](hv::TimerID id) {
        if (channel->isConnected() && channel->isWriteComplete()) {
          char str[DATETIME_FMT_BUFLEN] = {0};
          datetime_t dt = datetime_now();
          datetime_fmt(&dt, str);
          channel->send(str);
        } else {
          hv::killTimer(id);
        }
      });
    };

    service_.onmessage = [](const WebSocketChannelPtr& channel,
                            const std::string& msg) {
      auto ctx = channel->getContextPtr<Context>();
      ctx->HandleMessage(msg);
    };

    service_.onclose = [](const WebSocketChannelPtr& channel) {
      auto ctx = channel->getContextPtr<Context>();
      if (ctx->timer != INVALID_TIMER_ID) {
        hv::killTimer(ctx->timer);
        ctx->timer = INVALID_TIMER_ID;
      }
      // channel->deleteContextPtr();
    };

    ws_.service = &http;
    ws_.ws = &service_;
    websocket_server_run(&ws_, 1);
  }

 public:
  hv::WebSocketService service_;
  bool prepared_;
  DataType data_;

  WebServer() {
    Init();
    SPDLOG_TRACE("Constructed.");
  }

  explicit WebServer(const std::string& port_string) {
    Init();
    Open(port_string);
    SPDLOG_TRACE("Constructed.");
  }

  ~WebServer() {
    Close();
    SPDLOG_TRACE("Destructed");
  }

  void Open(const std::string& port_string) {
    port_ = std::atoi(port_string.c_str());
    ws_.port = port_;
  }

  void SetInterval(int interval) {
    if (interval > 0) interval_ = interval;
  }

  void Send(const std::string& message) {
    WebSocketChannelPtr channel;
    // channel->write(message);
    channel->send(message);
  }

  void Run() {
    while (!prepared_) {
      OpenPrepare();
    }
    Prepare();
  }

  void Close() { websocket_server_stop(&ws_); }
};
