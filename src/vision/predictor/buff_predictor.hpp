#pragma once

#include <chrono>
#include <vector>

#include "buff.hpp"
#include "buff_detector.hpp"
#include "common.hpp"
#include "ekf.hpp"
#include "kalman.hpp"
#include "opencv2/opencv.hpp"
#include "predictor.hpp"

struct BuffPredictorParam {
  bool is_EKF;
  EKF::Matx55d Q_mat;
  EKF::Matx33d R_mat;
  EKF::Matx55d Q_AC_mat;
  EKF::Matx33d R_AC_mat;
  bool is_KF;
  double delay_time;
  int error_frame;
};

class BuffPredictor : public Predictor<Armor, BuffPredictorParam, Kalman> {
 private:
  game::Race race_;
  component::BuffState state_;

  Buff buff_;
  std::size_t num_;
  std::chrono::system_clock::time_point end_time_;
  std::vector<cv::Point2f> circumference_;
  std::chrono::milliseconds duration_direction_, duration_predict_;

  void InitDefaultParams(const std::string &path);
  bool PrepareParams(const std::string &path);

  /**
   * @brief 匹配旋转方向
   *
   */
  void MatchDirection();

  /**
   * @brief 匹配预测器，根据赛种选择不同的能量机关预测方式
   *
   */
  void MatchPredict();

  /**
   * @brief 根据原装甲板和能量机关中心夹角角度模拟旋转装甲板
   *
   * @param theta 旋转角度
   * @return Armor 旋转后装甲板
   */
  Armor RotateArmor(double theta);

  /**
   * @brief 大符预测
   *
   */
  void BigBuffPredict();

  /**
   * @brief 小符预测
   *
   */
  void SmallBuffPredict();

 public:
  /**
   * @brief Construct a new BuffPredictor object
   *
   */
  BuffPredictor();

  /**
   * @brief Construct a new Buff Predictor object
   *
   * @param param 参数文件路径
   * @param buffs 传入的每帧得到的Buff
   */
  BuffPredictor(const std::string &param, const std::vector<Buff> &buffs);

  /**
   * @brief Destroy the BuffPredictor object
   *
   */
  ~BuffPredictor();

  /**
   * @brief Get the Buff object
   *
   * @return const Buff& 返回buff_
   */
  const Buff &GetBuff() const;

  /**
   * @brief Set the Buff object
   *
   * @param buff 传入buff_
   */
  void SetBuff(const Buff &buff);

  /**
   * @brief Get the State object
   *
   * @return component::BuffState& 当前能量机关旋转状态
   */
  component::BuffState &GetState();

  /**
   * @brief Set the State object
   *
   * @param state 当前能量机关旋转状态
   */
  void SetState(component::BuffState state);

  /**
   * @brief Get the Time object
   *
   * @return double 得到当前时间
   */
  double GetTime() const;

  /**
   * @brief Set the Time object
   *
   * @param time 传入当前时间
   */
  void SetTime(double time);

  /**
   * @brief 重新计时
   *
   */
  void ResetTime();

  /**
   * @brief Set the Race object
   *
   */
  void SetRace(game::Race race);

  /**
   * @brief 预测主函数
   *
   * @return std::vector<Armor> 返回预测装甲板
   */
  const std::vector<Armor> &Predict();

  /**
   * @brief 绘图函数
   *
   * @param output 所绘制图像
   * @param add_lable 标签等级
   */
  void VisualizePrediction(const cv::Mat &output, int add_lable);
};