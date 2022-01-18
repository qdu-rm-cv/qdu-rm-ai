#include "buff_predictor.hpp"

#include <ctime>

#include "common.hpp"

using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

#define RMU2022

namespace {

const auto kCV_FONT = cv::FONT_HERSHEY_SIMPLEX;
const cv::Scalar kGREEN(0., 255., 0.);
const cv::Scalar kRED(0., 0., 255.);
const cv::Scalar kYELLOW(0., 255., 255.);

const double kRMUT_TIME = 90.;
const double kRMUC_TIME = 420.;
const double kDELTA = 3;  //总延迟时间

}  // namespace

/**
 * @brief 辅助函数：旋转角角度计算
 *
 * @param p 圆周
 * @param ctr 圆心
 * @return double 旋转角(-pi~pi)
 */
static double CalRotatedAngle(const cv::Point2f &p, const cv::Point2f &ctr) {
  auto rel = p - ctr;
  return std::atan2(rel.x, rel.y);
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
#else
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
  SPDLOG_WARN("[BuffPredictor][InitDefaultParams] filter method: {}",
              filter_.method_);
  if (filter_.method_ != Method::kUNKNOWN) {
    if (filter_.method_ == Method::kEKF) {
      fs << "is_EKF" << true;
      fs << "Q_mat" << EKF::Matx55d::eye();
      fs << "R_mat" << EKF::Matx33d::eye();
      fs << "Q_AC_mat" << EKF::Matx55d::eye();
      fs << "R_AC_mat" << EKF::Matx33d::eye();
      fs << "is_KF" << false;
    } else if (filter_.method_ == Method::kKF) {
      SPDLOG_WARN("[BuffPredictor][InitDefaultParams] write kf param");
      fs << "is_EKF" << false;
      fs << "Q_mat" << EKF::Matx55d::zeros();
      fs << "R_mat" << EKF::Matx33d::zeros();
      fs << "Q_AC_mat" << EKF::Matx55d::zeros();
      fs << "R_AC_mat" << EKF::Matx33d::zeros();
      fs << "is_KF" << true;
    }
    fs << "delay_time" << 0.1542;
    fs << "error_frame" << 5;
  }
  SPDLOG_DEBUG("Inited params.");
}

bool BuffPredictor::PrepareParams(const std::string &params_path) {
  cv::FileStorage fs(params_path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);

  if (fs.isOpened()) {
    params_.is_EKF = (int)fs["is_EKF"] != 0 ? true : false;
    params_.Q_mat = fs["Q_mat"].mat();
    params_.R_mat = fs["R_mat"].mat();
    params_.Q_AC_mat = fs["Q_AC_mat"].mat();
    params_.R_AC_mat = fs["R_AC_mat"].mat();
    params_.is_KF = (int)fs["is_KF"] != 0 ? true : false;
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
  const auto start = std::chrono::system_clock::now();
  SPDLOG_WARN("start MatchDirection");

  if (direction_ == component::Direction::kUNKNOWN) {
    cv::Point2f center = buff_.GetCenter();
    double angle, sum = 0;
    std::vector<double> angles;

    if (circumference_.size() == 5) {
      for (auto point : circumference_) {
        angle = CalRotatedAngle(point, center);
        angles.emplace_back(angle);
      }

      for (auto i = circumference_.size(); i > circumference_.size() - 4; i--) {
        double delta = angles[i] - angles[i - 1];
        sum += delta;
      }

      if (sum > 0)
        direction_ = component::Direction::kCCW;
      else if (sum == 0)
        direction_ = component::Direction::kUNKNOWN;
      else
        direction_ = component::Direction::kCW;

      circumference_.emplace_back(buff_.GetTarget().ImageCenter());
      SPDLOG_DEBUG("Back of circumference point {}, {}.",
                   buff_.GetTarget().ImageCenter().x,
                   buff_.GetTarget().ImageCenter().y);
    }

    SPDLOG_WARN("Buff's Direction is {}", /* TODO */
                component::DirectionToString(direction_));
    const auto stop = std::chrono::system_clock::now();
    duration_direction_ =
        duration_cast<std::chrono::milliseconds>(stop - start);
  }
}

/**
 * @brief 根据原装甲板和能量机关中心夹角角度模拟旋转装甲板
 *
 * @param theta 旋转角度
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
  const auto start = std::chrono::system_clock::now();
  if (cv::Point2f(0, 0) == buff_.GetCenter()) {
    SPDLOG_ERROR("Center is empty.");
    return;
  }
  if (cv::Point2f(0, 0) == buff_.GetTarget().ImageCenter()) {
    SPDLOG_ERROR("Target center is empty.");
    return;
  }
  if (component::Direction::kUNKNOWN == direction_) return;
  component::BuffState state = GetState();
  if (state == component::BuffState::kSMALL) {
    SmallBuffPredict();
  } else if (state == component::BuffState::kBIG) {
    BigBuffPredict();
  }

  const auto stop = std::chrono::system_clock::now();
  duration_predict_ = duration_cast<std::chrono::milliseconds>(stop - start);
}

/**
 * @brief 大符预测
 *
 */
void BuffPredictor::BigBuffPredict() {
  predicts_.clear();
  cv::Mat frame(cv::Size(640, 480), CV_8UC3);
  cv::Point2d target_center = buff_.GetTarget().ImageCenter();
  cv::Point2d predicts_pt = filter_.Predict(target_center, frame);
  double theta = CalRotatedAngle(predicts_pt, target_center);
  Armor armor = RotateArmor(theta);
  predicts_.emplace_back(armor);
  SPDLOG_WARN("BigBuff has been predicted.");
}

/**
 * @brief 小符预测
 *
 */
void BuffPredictor::SmallBuffPredict() {
  predicts_.clear();
  cv::Point2f center = buff_.GetCenter();
  SPDLOG_DEBUG("center is {},{}", center.x, center.y);

  double theta = PredictIntegralRotatedAngle(GetTime());
  if (direction_ == component::Direction::kCW) theta = -theta;
  theta = theta / 180 * CV_PI;
  SPDLOG_WARN("Delta theta : {}", theta);
  Armor armor = RotateArmor(theta);
  predicts_.emplace_back(armor);
  SPDLOG_WARN("SmallBuff has been predicted.");
}

/**
 * @brief Construct a new Buff Predictor:: Buff Predictor object
 *
 */
BuffPredictor::BuffPredictor() { SPDLOG_TRACE("Constructed."); }

/**
 * @brief Construct a new Buff Predictor:: Buff Predictor object
 *
 * 1st. 初始化后不必修改
 *    1. filter 2. param
 * 2ed. 需要robot设置
 *    3. race   4. end_time
 * 3rd. 需要每帧更新
 *    5. buff   6. state    7.num   8.circumference
 *
 * @param param 参数文件路径
 */
BuffPredictor::BuffPredictor(const std::string &param) {
  SPDLOG_WARN("[BuffPredictor][Construct] Start construct");
#if 0
  //* 1. filter init
  if (filter_.method_ == Method::kUNKNOWN) {
    if (params_.is_EKF)
      filter_.method_ = Method::kEKF;
    else if (params_.is_KF)
      filter_.method_ = Method::kKF;
  }
  SPDLOG_WARN("[BuffPredictor][Construct] filter method : {}", filter_.method_);
  std::vector<double> init_vec;
  if (filter_.method_ == Method::kKF) {
    init_vec.push_back(4.);
    init_vec.push_back(2.);
  } else if (filter_.method_ == Method::kEKF) {
    for (std::size_t i = 0; i < 5; i++) init_vec.push_back(0.);
  }
  filter_.Init(init_vec);
  SPDLOG_WARN("[BuffPredictor][Construct] filter init");
#else
  //* 1st. filter init
  std::vector<double> init_vec;
  init_vec.push_back(4.);
  init_vec.push_back(2.);
  filter_.Init(init_vec);
  SPDLOG_WARN("[BuffPredictor][Construct] filter init");
#endif
  LoadParams(param);
  SPDLOG_WARN("[BuffPredictor][Construct] param init");

  //* 2nd. robot relation init
  race_ = game::Race::kUNKNOWN;
  SetTime(-200);
  SPDLOG_WARN("[BuffPredictor][Construct] race and end_time init");

  //* 3rd. buff init
  state_ = component::BuffState::kUNKNOWN;
  buff_ = Buff();
  num_ = 0;
  circumference_.clear();
  SPDLOG_WARN("[BuffPredictor][Construct] buff init");

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
  num_ = buff_.GetArmors().size();
  if (circumference_.size() < 5) {
    circumference_.push_back(buff_.GetTarget().ImageCenter());
    SPDLOG_DEBUG("Get Buff Center{},{} ", buff_.GetTarget().ImageCenter().x,
                 buff_.GetTarget().ImageCenter().y);
  }
}

/**
 * @brief Get the State object
 *
 * @return component::BuffState& 当前能量机关旋转状态
 */
component::BuffState &BuffPredictor::GetState() {
  if (race_ == game::Race::kRMUT) {
    state_ = component::BuffState::kBIG;
  } else if (race_ == game::Race::kRMUC) {
    int t = (int)(GetTime() / 60);
    switch (t) {
      case 0:
        state_ = component::BuffState::kINVINCIBLE;
        break;
      case 1:
      case 2:
        state_ = component::BuffState::kSMALL;
        break;
      case 3:
        state_ = component::BuffState::kINVINCIBLE;
        break;
      case 4:
      case 5:
      case 6:
        state_ = component::BuffState::kBIG;
        break;
      default:
        state_ = component::BuffState::kUNKNOWN;
        break;
    }
  }
  SPDLOG_DEBUG("Now state : {}", component::BuffStateToString(state_));
  return state_;
}

/**
 * @brief Set the State object
 *
 * @param state 当前能量机关旋转状态
 */
void BuffPredictor::SetState(component::BuffState state) {
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
  double game_time = 0;
  game::Race race_ = game::Race::kUNKNOWN;
  if (race_ == game::Race::kRMUT)
    game_time = kRMUT_TIME;
  else if (race_ == game::Race::kRMUC)
    game_time = kRMUC_TIME;

  auto duration = game_time - time;
  SPDLOG_WARN("duration : {}", duration);
  auto now = high_resolution_clock::now();
  auto end = now + std::chrono::seconds((int64_t)duration);
  end_time_ = end;

  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  SPDLOG_WARN("Now Ctime : {}", std::ctime(&now_time));
  SPDLOG_WARN("End Ctime : {}", std::ctime(&end_time));
}

/**
 * @brief 重新计时
 *
 */
void BuffPredictor::ResetTime() {
  if (buff_.GetArmors().size() < num_) SetTime(0);
  SPDLOG_WARN("Reset time.");
}

/**
 * @brief Set the Race object
 *
 * @param race 当前赛制
 */
void BuffPredictor::SetRace(game::Race race) {
  race_ = race;
  SPDLOG_DEBUG("Race type : {}", game::RaceToString(race));
}

/**
 * @brief 预测主函数
 *
 * @return std::vector<Armor> 返回预测装甲板
 */
const std::vector<Armor> &BuffPredictor::Predict() {
  SPDLOG_DEBUG("Predicting.");
  MatchDirection();
  MatchPredict();
  SPDLOG_ERROR("Predicted.");
  return predicts_;
}

/**
 * @brief 绘图函数
 *
 * @param output 所绘制图像
 * @param add_lable 标签等级
 */
void BuffPredictor::VisualizePrediction(const cv::Mat &output, int add_lable) {
  auto predict = predicts_.front();

  SPDLOG_DEBUG("{}, {}", predict.ImageCenter().x, predict.ImageCenter().y);

  if (add_lable > 0) {
    auto vertices = predict.ImageVertices();
    for (std::size_t i = 0; i < vertices.size(); ++i)
      cv::line(output, vertices[i], vertices[(i + 1) % 4], kYELLOW, 8);
    std::ostringstream buf;
    buf << predict.ImageCenter().x << ", " << predict.ImageCenter().y;
    cv::putText(output, buf.str(), vertices[1], kCV_FONT, 1.0, kRED);
  }
  if (add_lable > 1) {
    if (cv::Point2f(0, 0) != predict.ImageCenter())
      cv::line(output, buff_.GetCenter(), predict.ImageCenter(), kRED, 3);
  }
  if (add_lable > 2) {
    std::string label;
    int baseLine, v_pos = 0;

    label = cv::format("Direction %s in %ld ms.",
                       component::DirectionToString(direction_).c_str(),
                       duration_direction_.count());
    cv::Size text_size = cv::getTextSize(label, kCV_FONT, 1.0, 2, &baseLine);
    v_pos += 3 * static_cast<int>(1.3 * text_size.height);
    cv::putText(output, label, cv::Point(0, v_pos), kCV_FONT, 1.0, kGREEN);

    label = cv::format("Find predict in %ld ms.", duration_predict_.count());
    v_pos += static_cast<int>(1.3 * text_size.height);
    cv::putText(output, label, cv::Point(0, v_pos), kCV_FONT, 1.0, kGREEN);
  }
}
