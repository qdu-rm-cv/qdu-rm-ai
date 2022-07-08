#pragma once

#include <string>
#include <vector>

#include "libusbp-1/libusbp.hpp"

class USB {
 private:
  std::string port_name_;
  uint16_t vid_;  // vendor_id
  uint16_t pid_;  // product_id

 public:
  USB();
  USB(const uint16_t& vid, const uint16_t& pid);
  ~USB();

  const std::string& GetPortName();
  const std::string ConnectPortName();
  const std::string ConnectPortName(const uint16_t& vid, const uint16_t& pid);
};

const std::vector<USB> AutoList();
