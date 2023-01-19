#pragma once

#include <random>
#include <string>

#define RMU2023

#ifndef EnumName
#define EnumName(var) (std::string(#var))
#endif

#ifndef TypeName
#define TypeName(var) \
  std::string(abi::__cxa_demangle(typeid(var).name(), 0, 0, 0))
#endif
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
  kRMUL1V1,
  kRMUL3V3,
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

Model StringToModel(std::string name);

bool HasBigArmor(Model model);

}  // namespace game

namespace component {

struct Euler {
  double pitch, roll, yaw;

  explicit Euler(double pitch = 0, double roll = 0, double yaw = 0);
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

std::string ToString(const Euler &elur);
std::string ToString(const Direction &direction);
std::string ToString(const BuffState &state);
std::string ToString(const AimMethod &method);
std::string ToString(const game::Team &team);
std::string ToString(const game::Model &model);
std::string ToString(const game::Arm &arm);
std::string ToString(const game::Race &race);
std::string ToString(const game::RFID &rfid);
std::string ToString(const game::Method &m);
[[deprecated("The gtk-ui is recommended")]] AimMethod StringToAimMethod(
    std::string name);

}  // namespace component

namespace algo {

double RelativeDifference(double a, double b);

int GetIntRandomValue(int min = 0, int max = 10);
double GetRealRandomValue(double min = 0, double max = 1);

}  // namespace algo
