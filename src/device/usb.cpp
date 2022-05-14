#include "usb.hpp"

#include "spdlog/spdlog.h"

namespace {

const uint16_t kVID = 0xF407;
const uint16_t kPID = 0x4001;
const uint8_t kINTERFACE = 0;
const bool kCOMPOSITE = true;

}  // namespace

USB::USB() {
  port_name_ = ConnectPortName(kVID, kPID);
  SPDLOG_TRACE("Constructed");
}

USB::USB(const uint16_t &vid, const uint16_t &pid) {
  vid_ = vid;
  pid_ = pid;
  port_name_ = ConnectPortName(vid, pid);
  SPDLOG_TRACE("Constructed");
}

USB::~USB() { SPDLOG_TRACE("Destructed"); }

const std::string &USB::GetPortName() { return port_name_; }

const std::string USB::ConnectPortName(const uint16_t &vid,
                                       const uint16_t &pid) {
  libusbp::device device = libusbp::find_device_with_vid_pid(vid, pid);
  if (!device) {
    SPDLOG_ERROR("Cannot find device with vid:{}, pid:{}", vid, pid);
  }
  libusbp::serial_port port(device, kINTERFACE, kCOMPOSITE);
  port_name_ = port.get_name();
  return port_name_;
}

const std::vector<USB> AutoList() {
  std::vector<libusbp::device> list = libusbp::list_connected_devices();
  std::vector<USB> devices(list.size());

  for (auto it = list.begin(); it != list.end(); ++it) {
    uint16_t vendor_id = it->get_vendor_id();
    uint16_t product_id = it->get_product_id();
    USB usb(vendor_id, product_id);
    devices.emplace_back(usb);
    SPDLOG_DEBUG("vid : {}, pid : {}, portname : {}", vendor_id, product_id,
                 usb.GetPortName());
  }

  return devices;
}