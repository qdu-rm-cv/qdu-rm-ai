#pragma once

#include <string>

#define RMU2023

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
  kLIGHT,
};

std::string DirectionToString(Direction direction);
std::string BuffStateToString(BuffState state);
std::string AimMethodToString(AimMethod method);
[[deprecated("The ui approach is recommended")]] AimMethod StringToAimMethod(
    std::string name);

}  // namespace component

namespace game {

struct Alert {
  bool enemy_buff;
  bool enemy_snipe;
  bool enemy_slope;
  bool self_outpost;
  bool self_sentry;
  bool self_base;

  Alert() {
    enemy_buff = false;
    enemy_snipe = false;
    enemy_slope = false;
    self_outpost = false;
    self_sentry = false;
    self_base = false;
  }
};

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
  kBUFF,
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

enum class Method {
  kUNKNOWN,
  kKF,
  kEKF,
};

std::string TeamToString(Team team);
std::string ModelToString(Model model);
std::string ArmToString(Arm arm);
std::string RaceToString(Race race);
std::string RFIDToString(RFID rfid);
std::string MethodToString(const Method& m);
Model StringToModel(std::string name);

bool HasBigArmor(Model model);

}  // namespace game

namespace algo {

double RelativeDifference(double a, double b);

}  // namespace algo
