#include "aim_assitant.hpp"

AimAssitant::AimAssitant() { SPDLOG_TRACE("Constructed."); }

AimAssitant::AimAssitant(game::Model model) {
  model_ = model;
  SPDLOG_TRACE("Constructed.");
}

AimAssitant::~AimAssitant() { SPDLOG_TRACE("Destructed."); }

void AimAssitant::SetRFID(game::RFID rfid) {
  if (model_ == game::Model::kUNKNOWN) {
    method_ = component::AimMethod::kUNKNOWN;
    return;
  }

  if (model_ == game::Model::kHERO) {
    if (rfid == game::RFID::kSNIPE) {
      method_ = component::AimMethod::kSNIPE;
    } else if (rfid == game::RFID::kUNKNOWN) {
      method_ = component::AimMethod::kARMOR;
    }
  } else if (model_ == game::Model::kINFANTRY) {
    if (rfid == game::RFID::kBUFF) {
      method_ = component::AimMethod::kBUFF;
    } else if (rfid == game::RFID::kUNKNOWN) {
      method_ = component::AimMethod::kARMOR;
    }
  }
  SPDLOG_INFO("Now Arms : {}, AimMethod : {}", game::ModelToString(model_),
              component::AimMethodToString(method_));
}

void AimAssitant::Init(const std::string& armor_param,
                       const std::string& buff_param,
                       const std::string& snipe_param, game::Team enemy_team) {
  a_detector_.LoadParams(armor_param);
  b_detector_.LoadParams(buff_param);
  s_detector_.LoadParams(snipe_param);
  a_detector_.SetEnemyTeam(enemy_team);
  b_detector_.SetTeam(enemy_team);
  s_detector_.SetEnemyTeam(enemy_team);
}

const tbb::concurrent_vector<Armor>& AimAssitant::Detect(const cv::Mat& frame) {
  if (method_ == component::AimMethod::kUNKNOWN) {
    method_ = component::AimMethod::kARMOR;
  }
  if (method_ == component::AimMethod::kARMOR)
    return a_detector_.Detect(frame);
  else if (method_ == component::AimMethod::kBUFF) {
    auto buffs = b_detector_.Detect(frame);
    tbb::concurrent_vector<Armor> vec;
    for (auto& buff : buffs) {
      vec.push_back(buff.GetTarget());
    }
    return vec;
  } else if (method_ == component::AimMethod::kSNIPE) {
    return s_detector_.Detect(frame);
  }
}

void AimAssitant::VisualizeResult(const cv::Mat& frame, int add_label) {
  if (method_ == component::AimMethod::kARMOR)
    a_detector_.VisualizeResult(frame, add_label);
  else if (method_ == component::AimMethod::kBUFF) {
    b_detector_.VisualizeResult(frame, add_label);
  } else if (method_ == component::AimMethod::kSNIPE) {
    s_detector_.VisualizeResult(frame, add_label);
  }
}
