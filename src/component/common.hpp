#pragma once

#include <string>

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

std::string DirectionToString(Direction direction);
std::string BuffStateToString(BuffState state);

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

enum class Race {
  kUNKNOWN,
  kRMUT,
  kRMUL,
  kRMUC,
};

std::string TeamToString(Team team);
std::string ModelToString(Model model);
Model StringToModel(std::string name);

bool HasBigArmor(Model model);

}  // namespace game

namespace algo {

double RelativeDifference(double a, double b);

}  // namespace algo
