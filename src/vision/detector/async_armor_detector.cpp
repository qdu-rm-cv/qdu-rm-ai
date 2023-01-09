#include "async_armor_detector.hpp"

namespace {

const int kMAX_VECTOR_SIZE = 80;

}

ArmorDetectorAsync::ArmorDetectorAsync(const size_t thread_count) {
  thread_count_ = thread_count;

  method_vec_.clear();
  for (size_t i = 0; i < thread_count_; i++) {
    method_vec_.emplace_back(new ArmorDetector());
  }
}

ArmorDetectorAsync::~ArmorDetectorAsync() {
  thread_continue = false;
  for (size_t i = 0; i < threads_.size(); i++) {
    threads_.at(i)->join();
    delete threads_[i];
  }
}

void ArmorDetectorAsync::LoadParams(const std::string &path) {
  for (size_t i = 0; i < thread_count_; i++) {
    method_vec_[i]->LoadParams(path);
  }
}

void ArmorDetectorAsync::SetEnemyTeam(game::Team enemy_team) {
  for (size_t i = 0; i < thread_count_; i++) {
    method_vec_[i]->SetEnemyTeam(enemy_team);
  }
}

void ArmorDetectorAsync::Work(const size_t i) {
  while (true) {
    if (!thread_continue) return;

    source_signal_.Wait();
    mutex_source_.lock();
    cv::Mat frame = source_.front();
    source_.pop_front();
    mutex_source_.unlock();

    auto result = method_vec_[i]->Detect(frame);

    method_vec_[i]->VisualizeResult(frame, 5);
    cv::imshow(std::to_string(i), frame);
    cv::waitKey(1);

    if (result.size() == 0) continue;

    mutex_result_.lock();
    result_.emplace_back(result);
    if (result_.size() > kMAX_VECTOR_SIZE) {
      result_.pop_front();
    }
    mutex_result_.unlock();
  }
}

void ArmorDetectorAsync::PutFrame(const cv::Mat &frame) {
  mutex_source_.lock();
  source_.emplace_front(frame);
  if (source_.size() > kMAX_VECTOR_SIZE) {
    source_.pop_back();
  } else {
    source_signal_.Signal();
  }
  mutex_source_.unlock();
}

bool ArmorDetectorAsync::GetResult(tbb::concurrent_vector<Armor> &armors) {
  std::lock_guard<std::mutex> lock(mutex_result_);
  if (result_.size() == 0) return false;
  armors = result_.front();
  result_.pop_front();
  return true;
}
