#pragma once

#include <chrono>
#include <vector>

#include "buff.hpp"
#include "buff_detector.hpp"
#include "common.hpp"
#include "ekf.hpp"
#include "opencv2/opencv.hpp"
#include "predictor.hpp"

struct BuffPredictorEKFParam {
  EKF::Matx55d Q_mat;
  EKF::Matx33d R_mat;
  EKF::Matx55d Q_AC_mat;
  EKF::Matx33d R_AC_mat;
};

class BuffPredictor : public Predictor<Armor, BuffPredictorEKFParam, EKF> {
 private:
  Buff buff_;
  Armor predict_;
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
   * @brief 通过积分运算，原始装甲板数据，计算预测装甲板信息
   *
   */
  void MatchPredict();

  /**
   * @brief 根据原装甲板和角度模拟旋转装甲板
   *
   * @param armor 原始装甲板
   * @param theta 旋转角度
   * @param center 旋转中心
   * @return Armor 旋转后装甲板
   */
  Armor RotateArmor(const Armor &armor, double theta,
                    const cv::Point2f &center);

 public:
  /**
   * @brief Construct a new BuffPredictor object
   *
   */
  BuffPredictor();

  /**
   * @brief Construct a new BuffPredictor object
   *
   * @param buffs 传入的每帧得到的Buff
   */
  BuffPredictor(const std::vector<Buff> &buffs);

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
  void VisualizePrediction(const cv::Mat &output, bool add_lable);
};
