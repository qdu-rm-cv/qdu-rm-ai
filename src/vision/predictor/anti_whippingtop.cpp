
#include "anti_whippingtop.hpp"

#include <chrono>
#include <iostream>
#include <vector>
#include <execution>

#include "armor_detector.hpp"
#include "common.hpp"
#include "opencv2/opencv.hpp"
#include "predictor.hpp"

void AntiWhippingTop::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);

  fs << "center_height_diff_low" << 300.0;
  fs << "center_height_diff_high" << 600.0;
  fs << "center_width_diff_low" << 20.0;
  fs << "center_width_diff_high" << 80.0;
  fs << "detector_param_th" << 10;
  fs << "whipping_top_param_th" << 2;
  fs << "missing_frame_param_th" << 22;
  SPDLOG_DEBUG("Inited params.");
}

bool AntiWhippingTop::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (fs.isOpened()) {
    params_.center_height_diff_low = fs["center_height_diff_low"];
    params_.center_height_diff_high = fs["center_height_diff_high"];
    params_.center_width_diff_low = fs["center_width_diff_low"];
    params_.center_width_diff_high = fs["center_width_diff_high"];

    params_.detector_param_th = fs["detector_param_th"];
    params_.whipping_top_param_th = fs["whipping_top_param_th"];
    params_.missing_frame_param_th = fs["missing_frame_param_th"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params...");
    return false;
  }
}

void AntiWhippingTop::LoadParams(const std::string &path) {
  if (!PrepareParams(path)) {
    InitDefaultParams(path);
    PrepareParams(path);
    SPDLOG_WARN("Can not find params file. Created and reloaded.");
  }
  SPDLOG_DEBUG("Params loaded.");
}



/**
 * @brief 绘图函数
 *
 * @param output 所绘制图像
 * @param add_label 标签等级
 */
void AntiWhippingTop::VisualizePrediction(const cv::Mat &output,
    int add_label) {
  if (target_center_ != cv::Point2f(0, 0)) {
    auto draw_armor = [&](Armor &armor) {
      armor.VisualizeObject(output, add_label > 0, draw::kYELLOW);
    };

    if (!predicts_.empty()) {
      std::for_each(std::execution::par_unseq, predicts_.begin(),
          predicts_.end(), draw_armor);
    }
    if (add_label > 1) {
     std::string label =
          cv::format("Find predict in %ld ms.", duration_predict_.Count());
      draw::VisualizeLabel(output, label, 3);
    }
  }
}

// TODO(C.Meng) :如果已进入反陀螺策略，优化运行速度，并加强反陀螺策略的逻辑
/**
 * @brief 反陀螺策略函数
 */
void AntiWhippingTop::AntiStrategy() {
  SPDLOG_DEBUG("Start anti whipping top strategy...");
  duration_strategy_.Start();
  target_armor_ = GetTargetArmor(pre_armor_.GetRect(), now_armor_.GetRect());
  duration_strategy_.Calc("AntiStrategy");
}


/**
 * @brief 判断小陀螺函数,两帧作为一个完整AntiWhippingTop类
 *
 * @param targets detects
 */
bool AntiWhippingTop::IsWhipping(
    const tbb::concurrent_vector<Armor>
        targets) {
  duration_iswhipping_.Start();
  if (targets.empty()) {
    missing_frame_param_++;
    if (missing_frame_param_ >= params_.missing_frame_param_th) {
      detector_param_ = 0;
      EndWhipping();
      return false;
    }
    SPDLOG_ERROR("could not find armors...");
  } else {
    /* 前装甲未初始化 */
    if (!is_pre_loaded_) {
      InitPreArmor(targets.front());
      SPDLOG_TRACE("The pre armor center loaded...");
      is_pre_loaded_ = 1;
    } else {
      cv::Point2f pre_center = pre_armor_.ImageCenter();
      cv::Point2f now_center_potential = targets.front().ImageCenter();
      float length_diff = abs(now_center_potential.y - pre_center.y);
      float width_diff = abs(now_center_potential.x - pre_center.x);
      SPDLOG_INFO("length_diff:{}", length_diff);
      // TODO(C.Meng) : classify
      if (length_diff > params_.center_height_diff_high ||
          width_diff > params_.center_width_diff_high) {
        EndWhipping();
        return false;
      }
      if (width_diff < params_.center_width_diff_low) {
          if (length_diff < params_.center_height_diff_low) {
            detector_param_++;                               // 持续识别系数+1
            if (detector_param_ >= params_.detector_param_th)
              missing_frame_param_ = 0;                        // 掉帧系数清零
          }
      } else if (length_diff > params_.center_height_diff_low) {
        float now_angle_potential = targets.front().GetRect().angle;
        float pre_angle = pre_armor_.GetRect().angle;
        SPDLOG_INFO("now_angle_potential={}, pre_angle={}",
          now_angle_potential, pre_angle);
        /* 保证是不同侧的装甲板 */
        if ((now_angle_potential>= 0 && pre_angle<= 0)||
          (now_angle_potential<= 0 && pre_angle>= 0) ) {
          whipping_top_param_++;
          is_pre_loaded_ = 0;
          InitNowArmor(targets.front());
          cv::Point2f now_center = now_armor_.ImageCenter();
          float center_y_diff =
             pre_center.y - now_center.y;
          target_center_ = cv::Point2f(
              (pre_center.x + now_center.x) / 2,
              (pre_center.y + now_center.y) / 2 - center_y_diff / 2.0);
        }
      }
    }
  }

  /* 最后一步都要进行陀螺系数的判断 */

  if (whipping_top_param_ >= params_.whipping_top_param_th) {
    // GetRotation();
    AntiStrategy();
    duration_iswhipping_.Calc("IsWhipping");
    return true;
  } else {
    SPDLOG_TRACE("Not start the AntiStrategy...");
    duration_iswhipping_.Calc("IsWhipping");
    return false;
  }
}



/**
 * @brief 反陀螺预测主函数
 *
 * @param frame 图像帧
 *
 * @return tbb::concurrent_vector<Armor>& 预测装甲板
 */
const tbb::concurrent_vector<Armor>& AntiWhippingTop::Predict() {
  duration_predict_.Start();
  predicts_.clear();
  SPDLOG_WARN("Predicting.");
  is_whipping_ = IsWhipping(detects_);
  if (predicts_.size() == 0 && is_whipping_)
    predicts_.emplace_back(target_armor_);
  SPDLOG_WARN("Predicted.");
  duration_predict_.Calc();
  return predicts_;
}


/**
 * @brief 构建预测装甲板
 *
 * @param pre_rect 前旋转矩阵
 * @param now_rect 后旋转矩阵
 *
 * @return const Armor& 预测装甲板
 */
const Armor& AntiWhippingTop::GetTargetArmor(cv::RotatedRect pre_rect,
                                             cv::RotatedRect now_rect) {
  // TODO(MC) :长宽比还需要数学运算
  float width = (pre_rect.size.width+now_rect.size.width)/1.72;
  float height = (pre_rect.size.height+now_rect.size.height)/1.95;
  float angle = 0;
  // if (pre_rect.angle == 0 || now_rect.angle == 0)
  //   angle = 0;
  // else
  //   angle = (pre_rect.angle+now_rect.angle)/2;
  cv::RotatedRect target_rect(target_center_, cv::Size2f(width, height), angle);
  Armor trans(target_rect);
  target_armor_ = trans;
  return target_armor_;
}

/**
 * @brief 获取陀螺状态
 */
bool AntiWhippingTop::GetIsWhipping() {
  return is_whipping_;
}

/**
 * @brief 与自瞄的接口函数
 */
void AntiWhippingTop::SetDetects(
  const tbb::concurrent_vector<Armor> detects) {
  detects_ = detects;
}


bool AntiWhippingTop::ArmorClassify(int model) {  // 传入现在的model,判断松
  if (model == static_cast<int>(model_))
    return false;
  else if (!model)
    return false;
  else
    return true;
}

/**
 * @brief 提前结束反陀螺判断
 */
void AntiWhippingTop::EndWhipping() {
  whipping_top_param_ = 0;
  target_center_.x = 0, target_center_.y = 0;
  SPDLOG_TRACE("Not start the AntiStrategy...");
  duration_iswhipping_.Calc("IsWhipping");
}

AntiWhippingTop::AntiWhippingTop() { SPDLOG_TRACE("Constructed..."); }


AntiWhippingTop::AntiWhippingTop(const std::string params_path,
                                 game::Team enemy_team,
                                 const tbb::concurrent_vector<Armor> detects) {
  LoadParams(params_path);
  enemy_team_ = enemy_team;
  detects_ = detects;
  SPDLOG_TRACE("Constructed...");
}

AntiWhippingTop::~AntiWhippingTop() { SPDLOG_TRACE("Destructed..."); }


void AntiWhippingTop::InitPreArmor(const Armor armor_) {
  pre_armor_ = armor_;
  SPDLOG_TRACE("Init the pre armor...");
}

void AntiWhippingTop::InitNowArmor(const Armor armor_) {
  now_armor_ = armor_;
  SPDLOG_TRACE("Init the now armor...");
}

void AntiWhippingTop::SetEnemyTeam(game::Team enemy_team) {
  enemy_team_ = enemy_team;
}



