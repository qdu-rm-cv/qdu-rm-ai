#pragma once

#include <mutex>
#include <thread>

#include "armor.hpp"
#include "armor_detector.hpp"
#include "async.hpp"
#include "semaphore.hpp"

class ArmorDetectorAsync
    : public Async<cv::Mat, tbb::concurrent_vector<Armor>, ArmorDetector> {
 private:
  void Work(const size_t i);

 public:
  ArmorDetectorAsync(const size_t thread_count);

  ~ArmorDetectorAsync();

  void LoadParams(const std::string &path);

  void SetEnemyTeam(game::Team enemy_team);

  // void Start();

  // void Stop();

  /**
   * @brief 上传图片数据
   *
   * @param frame 图片
   */
  void PutFrame(const cv::Mat &frame);

  /**
   * @brief Get the Result object 下载结果
   *
   * @return tbb::concurrent_vector<Armor> armors
   */
  bool GetResult(tbb::concurrent_vector<Armor> &);
};