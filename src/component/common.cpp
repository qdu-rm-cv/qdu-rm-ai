#include "common.hpp"

#include <algorithm>
#include <cctype>
#include <string>

#include "spdlog/fmt/bundled/core.h"

namespace component {

Euler::Euler(double yaw, double pitch, double roll)
    : yaw(yaw), pitch(pitch), roll(roll) {}

const std::string Euler::ToString() {
  return fmt::format("yaw : {}, pitch : {}, roll : {}", yaw, pitch, roll);
}

std::string DirectionToString(Direction direction) {
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

std::string BuffStateToString(BuffState state) {
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

std::string AimMethodToString(AimMethod method) {
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

namespace game {

std::string TeamToString(Team team) {
  switch (team) {
    case Team::kUNKNOWN:
      return std::string("Unknown");
    case Team::kDEAD:
      return std::string("Dead");
    case Team::kBLUE:
      return std::string("Blue");
    case Team::kRED:
      return std::string("Red");
    default:
      return std::string("Unknown");
  }
}

std::string ModelToString(Model model) {
  switch (model) {
    case Model::kUNKNOWN:
      return std::string("Unknown");
    case Model::kINFANTRY:
      return std::string("Infantry");
    case Model::kHERO:
      return std::string("Hero");
    case Model::kENGINEER:
      return std::string("Engineer");
    case Model::kDRONE:
      return std::string("Drone");
    case Model::kSENTRY:
      return std::string("Sentry");
    case Model::kBASE:
      return std::string("Base");
    case Model::kOUTPOST:
      return std::string("Outpost");
    case Model::kBUFF:
      return std::string("Buff");
    default:
      return std::string("Unknown");
  }
}

std::string ArmToString(Arm arm) {
  switch (arm) {
    case Arm::kUNKNOWN:
      return std::string("Unknown");
    case Arm::kINFANTRY:
      return std::string("Infantry");
    case Arm::kHERO:
      return std::string("Hero");
    case Arm::kENGINEER:
      return std::string("Engineer");
    case Arm::kDRONE:
      return std::string("Drone");
    case Arm::kSENTRY:
      return std::string("Sentry");
    case Arm::kDART:
      return std::string("Dart");
    case Arm::kRADAR:
      return std::string("Radar");
    default:
      return std::string("Unknown");
  }
}

std::string RaceToString(Race race) {
  switch (race) {
    case Race::kUNKNOWN:
      return std::string("Unknown");
    case Race::kRMUC:
      return std::string("RMUC");
    case Race::kRMUT:
      return std::string("RMUT");
    case Race::kRMUL1:
      return std::string("RMUL 1v1");
    case Race::kRMUL3:
      return std::string("RMUL 3v3");
    default:
      return std::string("Unknown");
  }
}

std::string RFIDToString(RFID rfid) {
  switch (rfid) {
    case RFID::kUNKNOWN:
      return std::string("Unknown");
    case RFID::kBUFF:
      return std::string("Buff activation point");
    case RFID::kSNIPE:
      return std::string("Snipe point");
    default:
      return std::string("Unknown");
  }
}

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

std::string MethodToString(const Method& m) {
  switch (m) {
    case Method::kEKF:
      return std::string("Extend kalman filter");
    case Method::kKF:
      return std::string("Kalman filter");
    default:
      return std::string("Unknown");
  }
}

bool HasBigArmor(Model model) {
  return (model == Model::kHERO || model == Model::kSENTRY);
}

}  // namespace game

namespace algo {

double RelativeDifference(double a, double b) {
  double diff = std::abs(a - b);
  double base = std::max(std::abs(a), std::abs(b));
  return diff / base;
}

}  // namespace algo
