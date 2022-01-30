#pragma once

#include <string>

#include "spdlog/spdlog.h"

namespace component {

struct Euler {
  double yaw, pitch, roll;
};

enum class Direction {
  kUNKNOWN,
  kCW,
  kCCW,
};

enum class BuffState {
  kUNKNOWN,
  kSMALL,
  kBIG,
  kINVINCIBLE,
};

enum class AimMethod {
  kUNKNOWN,
  kARMOR,
  kBUFF,
  kORECUBE,
  kSNIPE,
};

std::string DirectionToString(Direction direction);
std::string BuffStateToString(BuffState state);
std::string AimMethodToString(AimMethod method);

}  // namespace component

namespace game {

enum class Team {
  kUNKNOWN,
  kDEAD,
  kBLUE,
  kRED,
};

enum class Model {
  kUNKNOWN,
  kINFANTRY,
  kHERO,
  kENGINEER,
  kDRONE,
  kSENTRY,
  kBASE,
  kOUTPOST,
};

enum class Arm {
  kUNKNOWN,
  kINFANTRY,
  kHERO,
  kENGINEER,
  kDRONE,
  kSENTRY,
  kDART,
  kRADAR,
};

enum class Race {
  kUNKNOWN,
  kRMUC,
  kRMUT,
  kRMUL1,
  kRMUL3,
};

enum class RFID {
  kUNKNOWN,
  kBUFF,
  kSNIPE,
};

std::string TeamToString(Team team);
std::string ModelToString(Model model);
std::string ArmToString(Arm arm);
std::string RaceToString(Race race);
std::string RFIDToString(RFID rfid);
Model StringToModel(std::string name);

bool HasBigArmor(Model model);

}  // namespace game

namespace algo {

double RelativeDifference(double a, double b);

}  // namespace algo

namespace RMlogger {

const std::string fmt0("%+");
const std::string fmt1("[%Y-%m-%d %T.%3!u] %^[%l] [%!]%$ [%s:%#] %v");
const std::string fmt2(
    "[%Y-%m-%d %T.%3!u] %^[%l]%$ [%s:%#] \033[34m[%!]\033[0m %v");

void SetLogger(
    spdlog::level::level_enum level = spdlog::level::level_enum::debug,
    const std::string& fmt = fmt2);

}  // namespace RMlogger