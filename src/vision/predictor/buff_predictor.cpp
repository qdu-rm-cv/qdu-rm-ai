#include "buff_predictor.hpp"

#include <chrono>
#include <ctime>

#include "common.hpp"

using std::chrono::high_resolution_clock;

namespace {

const double kRMUT_TIME = 90.;
const double kRMUC_TIME = 420.;
const double kDELTA = 3;  // 总延迟时间

}  // namespace

/**
 * @brief 辅助函数：旋转角角度计算
 *
 * @param p 圆周
 * @param ctr 圆心
 * @return double 旋转角(0~2pi)
 */
static double CalRotatedAngle(const cv::Point2f &p, const cv::Point2f &ctr) {
  auto rel = p - ctr;
  return std::atan2(-rel.y, rel.x) + M_PI;
}

#ifdef RMU2021
/**
 * @brief 辅助函数：积分运算预测旋转角
 *
 * @param t 当前时刻
 * @return double 旋转角
 */
static double PredictIntegralRotatedAngle(double t) {
  return 1.305 * kDELTA +
         0.785 / 1.884 * (cos(1.884 * t) - cos(1.884 * (t + kDELTA)));
}
#elif RMU2022
/**
 * @brief 小符角速度 Omega = 10 RPM
 * \displaystyle{\omega = 10 \texttt{ RPM}} \\ \\
 * \displaystyle{\theta = \int_{t_{0}}^{t_{0}+\Delta t} \omega t dt = 10t
 * \bigg|_{t_{0}}}
 * @param t 当前时刻t0
 * @return double 积分预测角
 */
static double PredictIntegralRotatedAngle(double t) {
  return 5 * (2 * kDELTA + t) * t;
}
#endif

void BuffPredictor::InitDefaultParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
  SPDLOG_WARN("filter method: {}", game::ToString(filter_.method_));
  if (filter_.method_ != component::FilterMethod::kUNKNOWN) {
    fs << "delay_time" << 0.1542;
    fs << "error_frame" << 5;
  }
  SPDLOG_DEBUG("Inited params.");
}

bool BuffPredictor::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);

  if (fs.isOpened()) {
    params_.delay_time = fs["delay_time"];
    params_.error_frame = fs["error_frame"];
    return true;
  } else {
    SPDLOG_ERROR("Can not load params.");
    return false;
  }
}

/**
 * @brief 匹配旋转方向
 *
 */
void BuffPredictor::MatchDirection() {
  SPDLOG_WARN("Start MatchDirection");

  if (direction_ == component::Direction::kUNKNOWN) {
    duration_direction_.Start();
    cv::Point2f center = buff_.GetCenter();
    double angle, sum = 0;
    std::vector<double> angles;

    for (auto point : circumference_) {
      angle = CalRotatedAngle(point, center);
      angles.emplace_back(angle);
      SPDLOG_WARN("angle[i], {}", angle);
    }

    for (auto i = circumference_.size() - 1; i > circumference_.size() - 29;
         i--) {
      double delta = angles[i] - angles[i - 1];
      if (std::abs(delta) > M_PI / 4) delta = 0;
      SPDLOG_WARN("delta {}: {} - {} =  {},", i, angles[i], angles[i - 1],
                  delta);
      sum += delta;
    }

    if (sum > 0)
      direction_ = component::Direction::kCCW;  // 逆时针
    else if (sum == 0)
      direction_ = component::Direction::kUNKNOWN;
    else
      direction_ = component::Direction::kCW;  // 顺时针

    circumference_.emplace_back(buff_.GetTarget().ImageCenter());
    SPDLOG_WARN(" sum is {}", sum);
  } else if (circumference_.size() < 30) {
    circumference_.emplace_back(buff_.GetTarget().ImageCenter());
  } else {
    circumference_.erase(circumference_.begin());
  }
  SPDLOG_WARN("Buff's Direction is {}", game::ToString(direction_));
  duration_direction_.Calc("Predict Direction");
}

/**
 * @brief 根据原装甲板和能量机关中心夹角角度模拟旋转装甲板
 *
 * @param theta 变化的旋转角度
 * @return Armor 旋转后装甲板
 */
Armor BuffPredictor::RotateArmor(double theta) {
  cv::Point2f predict_point[4], center = buff_.GetCenter();
  cv::Matx22d rot(cos(theta), -sin(theta), sin(theta), cos(theta));

  auto vertices = buff_.GetTarget().ImageVertices();
  for (int i = 0; i < 3; i++) {
    cv::Matx21d vec(vertices[i].x - center.x, vertices[i].y - center.y);
    cv::Matx21d mat = rot * vec;
    predict_point[i] =
        cv::Point2f(mat.val[0] + center.x, mat.val[1] + center.y);
  }
  return Armor(
      cv::RotatedRect(predict_point[0], predict_point[1], predict_point[2]));
}

/**
 * @brief 匹配预测器，根据赛种选择不同的能量机关预测方式
 *
 */
void BuffPredictor::MatchPredict() {
  duration_predict_.Start();
  if (cv::Point2f(0, 0) == buff_.GetCenter()) {
    SPDLOG_ERROR("Center is empty.");
    return;
  }
  if (cv::Point2f(0, 0) == buff_.GetTarget().ImageCenter()) {
    SPDLOG_ERROR("Target center is empty.");
    return;
  }
  if (component::Direction::kUNKNOWN == direction_) return;
  game::BuffState state = GetState();
  double theta = 0;
  if (state == game::BuffState::kSMALL) {
    // theta = PredictIntegralRotatedAngle(2);
    theta = 10;
    theta = theta / 180 * CV_PI;
  } else if (state == game::BuffState::kBIG) {
    theta = std::abs(
        CalRotatedAngle(filter_.Predict(buff_.GetTarget().ImageCenter()),
                        buff_.GetCenter()) -
        CalRotatedAngle(buff_.GetTarget().ImageCenter(), buff_.GetCenter()));
  }
  if (direction_ == component::Direction::kCCW) theta = -theta;
  Armor armor = RotateArmor(theta);
  armor.SetModel(game::Model::kHERO);
  predicts_.emplace_back(armor);
  SPDLOG_WARN("Buff has been predicted. {}", game::ToString(direction_));

  duration_predict_.Calc("Match Predict");
}

/**
 * @brief Construct a new Buff Predictor:: Buff Predictor object
 *
 */
BuffPredictor::BuffPredictor() {
  std::vector<double> init_vec = {4., 2.};
  filter_.Init(init_vec);
  SPDLOG_DEBUG("filter init");
  race_ = game::Race::kUNKNOWN;
  SetTime(-200);
  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Construct a new Buff Predictor:: Buff Predictor object
 *
 * 1st. 初始化后不必修改
 *    1. filter 2. param
 * 2nd. 需要robot设置
 *    3. race   4. end_time
 * 3rd. 需要每帧更新
 *    5. buff   6. state    7.circumference
 *
 * @param param 参数文件路径
 */
BuffPredictor::BuffPredictor(const std::string &param) {
  SPDLOG_WARN("Start construct");
#if 0
  //* 1. filter init
  if (filter_.method_ == FilterMethod::kUNKNOWN) {
    if (params_.is_EKF)
      filter_.method_ = FilterMethod::kEKF;
    else if (params_.is_KF)
      filter_.method_ = FilterMethod::kKF;
  }
  SPDLOG_DEBUG("Filter method : {}", filter_.method_);
  std::vector<double> init_vec;
  if (filter_.method_ == FilterMethod::kKF) {
    init_vec.push_back(4.);
    init_vec.push_back(2.);
  } else if (filter_.method_ == FilterMethod::kEKF) {
    for (std::size_t i = 0; i < 5; i++) init_vec.push_back(0.);
  }
  filter_.Init(init_vec);
  SPDLOG_DEBUG("Filter init");
#else
  //* 1st. filter init
  std::vector<double> init_vec = {4., 2.};
  filter_.Init(init_vec);
  SPDLOG_INFO("Filter init");
#endif
  LoadParams(param);
  SPDLOG_INFO("Param init");

  //* 2nd. robot relation init
  race_ = game::Race::kUNKNOWN;
  SetTime(-200);
  SPDLOG_INFO("Race and end_time init");

  //* 3rd. buff init
  state_ = game::BuffState::kUNKNOWN;
  buff_ = Buff();
  circumference_.clear();
  SPDLOG_INFO("Buff init");

  SPDLOG_TRACE("Constructed.");
}

/**
 * @brief Destroy the Buff Predictor:: Buff Predictor object
 *
 */
BuffPredictor::~BuffPredictor() { SPDLOG_TRACE("Destructed."); }

/**
 * @brief Set the Buff object
 *
 * @param buff 传入buff_
 */
void BuffPredictor::SetBuff(const Buff &buff) {
  state_ = GetState();
  buff_ = buff;
  if (circumference_.size() < 5) {
    circumference_.push_back(buff_.GetTarget().ImageCenter());
    SPDLOG_DEBUG("Get Buff Center {},{} ", buff_.GetTarget().ImageCenter().x,
                 buff_.GetTarget().ImageCenter().y);
  }
}

void BuffPredictor::ChangeDirection(bool direction) {
  if (direction) {
    direction_ = component::Direction::kCW;
  } else {
    direction_ = component::Direction::kCCW;
  }
  SPDLOG_CRITICAL("Direction changed, now : {}", game::ToString(direction_));
}

/**
 * @brief Get the State object
 *
 * @return game::BuffState& 当前能量机关旋转状态
 */
game::BuffState &BuffPredictor::GetState() {
  SPDLOG_DEBUG("{}, {}", game::ToString(race_), game::ToString(state_));

  if (race_ == game::Race::kRMUT) {
    state_ = game::BuffState::kBIG;
  } else if (race_ == game::Race::kRMUC) {
    double t = GetTime();
    if (t < 1 * 60 || (t >= 3 * 60 && t < 4 * 60))
      state_ = game::BuffState::kINVINCIBLE;
    else if (t >= 1 * 60 && t < 3 * 60)
      state_ = game::BuffState::kSMALL;
    else if (t >= 4 * 60 && t < 7 * 60)
      state_ = game::BuffState::kBIG;
  }

  SPDLOG_DEBUG("Now state : {}", game::ToString(state_));
  return state_;
}

/**
 * @brief Set the State object
 *
 * @param state 当前能量机关旋转状态
 */
void BuffPredictor::SetState(game::BuffState state) {
  state_ = state;
  SPDLOG_DEBUG("State has been set.");
}

/**
 * @brief Get the Time object
 *
 * @return double 得到当前时间
 */
double BuffPredictor::GetTime() const {
  auto time = end_time_ - high_resolution_clock::now();
  auto now = time.count() / 1000000.;
  SPDLOG_WARN("time_: {}ms", now);
  return now;
}

/**
 * @brief Set the Time object
 *
 * @param time 传入当前时间
 */
void BuffPredictor::SetTime(double time) {
  auto now = high_resolution_clock::now();
  auto end = high_resolution_clock::time_point();
  if (time < 0) {
    end_time_ = now;
    SPDLOG_WARN("time < 0");
  }
  if (end_time_ == now) {
    double game_time = 0;
    if (race_ == game::Race::kUNKNOWN) {
      if (race_ == game::Race::kRMUT)
        game_time = kRMUT_TIME;
      else if (race_ == game::Race::kRMUC)
        game_time = kRMUC_TIME;
    }
    double duration = game_time - time;
    end = now + std::chrono::seconds((int64_t)duration);
    end_time_ = end;
  }
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  SPDLOG_WARN("Now Ctime : {}", std::ctime(&now_time));
  SPDLOG_WARN("End Ctime : {}", std::ctime(&end_time));
}

/**
 * @brief Set the Race object
 *
 * @param race 当前赛制
 */
void BuffPredictor::SetRace(game::Race race) {
  race_ = race;
  SPDLOG_DEBUG("Race type : {}", game::ToString(race));
}

/**
 * @brief 预测主函数
 *
 * @return tbb::concurrent_vector<Armor> 返回预测装甲板
 */
const tbb::concurrent_vector<Armor> &BuffPredictor::Predict() {
  predicts_.clear();
  SPDLOG_WARN("Predicting.");
  MatchDirection();
  MatchPredict();
  SPDLOG_WARN("Predicted.");
  if (predicts_.size() == 0) {
    predicts_.emplace_back(buff_.GetTarget());
  }
  return predicts_;
}

/**
 * @brief 绘图函数
 *
 * @param output 所绘制图像
 * @param add_lable 标签等级
 */
void BuffPredictor::VisualizePrediction(const cv::Mat &output, int add_lable) {
  for (auto predict : predicts_) {
    SPDLOG_DEBUG("{}, {}", predict.ImageCenter().x, predict.ImageCenter().y);

    if (add_lable > 0) {
      auto vertices = predict.ImageVertices();
      for (std::size_t i = 0; i < vertices.size(); ++i)
        cv::line(output, vertices[i], vertices[(i + 1) % 4], draw::kYELLOW, 2);
      std::string buf = cv::format("%.3f, %.3f", predict.ImageCenter().x,
                                   predict.ImageCenter().y);
      cv::putText(output, buf, vertices[1], draw::kCV_FONT, 1.0, draw::kRED);
    }
    if (add_lable > 1) {
      if (cv::Point2f(0, 0) != predict.ImageCenter())
        cv::line(output, buff_.GetCenter(), predict.ImageCenter(), draw::kRED,
                 2);
    }
  }
  if (add_lable > 2) {
    std::string label;
    if (direction_ == component::Direction::kUNKNOWN) {
      label = cv::format("Direction : %s in %ld ms.",
                         game::ToString(direction_).c_str(),
                         duration_direction_.Count());
    } else {
      label = cv::format("Direction : %s.", game::ToString(direction_).c_str());
    }
    draw::VisualizeLabel(output, label, 5);

    label = cv::format("Find predict in %ld ms.", duration_predict_.Count());
    draw::VisualizeLabel(output, label, 6);
  }
}
