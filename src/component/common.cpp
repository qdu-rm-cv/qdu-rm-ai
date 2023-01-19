#include "common.hpp"

#include <cxxabi.h>

#include <algorithm>
#include <cctype>
#include <string>

#include "spdlog/fmt/bundled/core.h"

namespace game {

Model StringToModel(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (!name.compare("infantry") || !name.compare("3") || !name.compare("4") ||
      !name.compare("5")) {
    return Model::kINFANTRY;
  }
  if (!name.compare("hero") || !name.compare("1")) {
    return Model::kHERO;
  }
  if (!name.compare("engineer") || !name.compare("2")) {
    return Model::kENGINEER;
  }
  if (!name.compare("drone")) {
    return Model::kDRONE;
  }
  if (!name.compare("sentry")) {
    return Model::kSENTRY;
  }
  if (!name.compare("base")) {
    return Model::kBASE;
  }
  if (!name.compare("outpost")) {
    return Model::kOUTPOST;
  }
  if (!name.compare("buff")) {
    return Model::kBUFF;
  }
  return Model::kUNKNOWN;
}

bool HasBigArmor(Model model) {
  return (model == Model::kHERO || model == Model::kSENTRY);
}

}  // namespace game

namespace component {

Euler::Euler(double pitch, double roll, double yaw)
    : pitch(pitch), roll(roll), yaw(yaw) {}

std::string ToString(const Euler& e) {
  return fmt::format("pitch : {}, roll : {}， yaw : {}", e.pitch, e.roll,
                     e.yaw);
}

std::string ToString(const Direction& direction) {
  switch (direction) {
    case Direction::kUNKNOWN:
      return std::string("Unknown");
    case Direction::kCW:
      return std::string("Clockwise");
    case Direction::kCCW:
      return std::string("Counterclockwise");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const BuffState& state) {
  switch (state) {
    case BuffState::kUNKNOWN:
      return std::string("Unknown");
    case BuffState::kSMALL:
      return std::string("Small Buff");
    case BuffState::kBIG:
      return std::string("Big Buff");
    case BuffState::kINVINCIBLE:
      return std::string("Can't be hit");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const AimMethod& method) {
  switch (method) {
    case AimMethod::kUNKNOWN:
      return std::string("Unknown");
    case AimMethod::kARMOR:
      return std::string("Use Armor Detector");
    case AimMethod::kBUFF:
      return std::string("Use Buff Detector");
    case AimMethod::kORECUBE:
      return std::string("Use OreCube Detector");
    case AimMethod::kSNIPE:
      return std::string("Use Snipe Detector");
    case AimMethod::kLIGHT:
      return std::string("Use GuidingLight Detector");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::Team& team) {
  switch (team) {
    case game::Team::kUNKNOWN:
      return std::string("Unknown");
    case game::Team::kDEAD:
      return std::string("Dead");
    case game::Team::kBLUE:
      return std::string("Blue");
    case game::Team::kRED:
      return std::string("Red");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::Model& model) {
  switch (model) {
    case game::Model::kUNKNOWN:
      return std::string("Unknown");
    case game::Model::kINFANTRY:
      return std::string("Infantry");
    case game::Model::kHERO:
      return std::string("Hero");
    case game::Model::kENGINEER:
      return std::string("Engineer");
    case game::Model::kDRONE:
      return std::string("Drone");
    case game::Model::kSENTRY:
      return std::string("Sentry");
    case game::Model::kBASE:
      return std::string("Base");
    case game::Model::kOUTPOST:
      return std::string("Outpost");
    case game::Model::kBUFF:
      return std::string("Buff");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::Arm& arm) {
  switch (arm) {
    case game::Arm::kUNKNOWN:
      return std::string("Unknown");
    case game::Arm::kINFANTRY:
      return std::string("Infantry");
    case game::Arm::kHERO:
      return std::string("Hero");
    case game::Arm::kENGINEER:
      return std::string("Engineer");
    case game::Arm::kDRONE:
      return std::string("Drone");
    case game::Arm::kSENTRY:
      return std::string("Sentry");
    case game::Arm::kDART:
      return std::string("Dart");
    case game::Arm::kRADAR:
      return std::string("Radar");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::Race& race) {
  switch (race) {
    case game::Race::kUNKNOWN:
      return std::string("Unknown");
    case game::Race::kRMUC:
      return std::string("RMUC");
    case game::Race::kRMUT:
      return std::string("RMUT");
    case game::Race::kRMUL1V1:
      return std::string("RMUL 1v1");
    case game::Race::kRMUL3V3:
      return std::string("RMUL 3v3");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::RFID& rfid) {
  switch (rfid) {
    case game::RFID::kUNKNOWN:
      return std::string("Unknown");
    case game::RFID::kBUFF:
      return std::string("Buff activation point");
    case game::RFID::kSNIPE:
      return std::string("Snipe point");
    default:
      return std::string("Unknown");
  }
}

std::string ToString(const game::Method& m) {
  switch (m) {
    case game::Method::kUNKNOWN:
      return std::string("Unknown");
    case game::Method::kEKF:
      return std::string("Extend kalman filter");
    case game::Method::kKF:
      return std::string("Kalman filter");
    default:
      return std::string("Unknown");
  }
}

[[deprecated("The gtk-ui is recommended")]] AimMethod StringToAimMethod(
    std::string name) {
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (!name.compare("armor") || !name.compare("autoaim") ||
      !name.compare("auto_aim") || !name.compare("auto-aim") ||
      !name.compare("1")) {
    return AimMethod::kARMOR;
  }
  if (!name.compare("buff") || !name.compare("2")) {
    return AimMethod::kBUFF;
  }
  if (!name.compare("orecube") || !name.compare("cube") ||
      !name.compare("ore") || !name.compare("3")) {
    return AimMethod::kORECUBE;
  }
  if (!name.compare("snipe") || !name.compare("4")) {
    return AimMethod::kSNIPE;
  }
  if (!name.compare("light") || !name.compare("guidinglight") ||
      !name.compare("guiding-light") || !name.compare("guiding_light") ||
      !name.compare("5")) {
    return AimMethod::kLIGHT;
  }
  return AimMethod::kUNKNOWN;
}

}  // namespace component

namespace algo {

static std::random_device r;
static std::default_random_engine engine(r());

double RelativeDifference(double a, double b) {
  double diff = std::abs(a - b);
  double base = std::max(std::abs(a), std::abs(b));
  return diff / base;
}

int GetIntRandomValue(int min, int max) {
  if (min > max) std::swap(min, max);
  std::uniform_int_distribution<int> distributer(min, max);
  return distributer(engine);
}

double GetRealRandomValue(double min, double max) {
  if (min > max) std::swap(min, max);
  static std::uniform_real_distribution<double> distributer(min, max);
  return distributer(engine);
}

}  // namespace algo
