#include "template_matcher.hpp"

[[deprecated("discard function")]] int TemplaterMatcher::GetArmorId(
    cv::Mat &armor_face) {
  cv::Mat img = armor_face;
  if (img.empty()) {
    SPDLOG_ERROR("Armor Face Empty!");
    return -1;
  }
  cv::Mat clone_img = img.clone();                       // 原图备份
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);            // 图片转灰度
  cv::threshold(img, img, 100, 255, cv::THRESH_BINARY);  // 二值化
  cv::Mat clone_img1 = img.clone();              // 备份二值化后的原图
  std::vector<std::vector<cv::Point>> contours;  // 存储原图轮廓
  std::vector<cv::Vec4i> hierarcy;
  cv::findContours(img, contours, hierarcy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);           // 查找轮廓
  std::vector<cv::Rect> sort_rect(contours.size());  // 外接矩形
  std::vector<cv::Mat> img_mat;  // 存储原图中的有效数字区域
  for (int i = 0; size_t(i) < contours.size(); i++) {
    sort_rect[i] = cv::boundingRect(contours[i]);  // 外接矩形
    cv::Mat roi = clone_img1(sort_rect[i]);
    cv::Mat dstroi;
    cv::resize(roi, dstroi, roi.size(), 0, 0);  // 重设大小
    img_mat.push_back(dstroi);
  }
  // 对矩形进行排序，因为轮廓的顺序不一定是数字真正的顺序
  for (int i = 0; size_t(i) < sort_rect.size(); i++) {
    for (int j = i + 1; size_t(j) < sort_rect.size(); j++) {
      if (sort_rect[j].x < sort_rect[i].x) {
        std::swap(img_mat[i], img_mat[j]);
      }
    }
  }
  std::vector<cv::Mat> templates;  // 模板容器
  for (int i = 0; i < 4; i++) {
    std::string name = cv::format("%s/template/%d.png", kPATH_IMAGE, i);
    cv::Mat temp = cv::imread(name);
    if (temp.empty()) {
      SPDLOG_ERROR("模板读取失败");
      return -1;
    }
    cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
    cv::threshold(temp, temp, 100, 255, cv::THRESH_BINARY);
    templates.push_back(temp);
  }
  if (templates.empty()) {
    SPDLOG_ERROR("模板导入失败");
    return -2;
  }
  if (img_mat.empty()) {
    SPDLOG_ERROR("二值化处理之后的源导入失败");
    return -2;
  }
  std::vector<int> seq;  // 顺序存放识别结果
  for (int i = 0; size_t(i) < img_mat.size(); i++) {
    double com = 0;
    double min = 0;
    int min_seq = 0;  // 记录识别结果
    for (int j = 0; size_t(j) < templates.size(); j++) {
      com = MatchTemplate(img_mat[i], templates[j]);
      if (com > min) {
        min = com;
        min_seq = j;
      }
      com = 0;
    }
    seq.push_back(min_seq);
  }
  if (!seq.empty()) return seq[0];
  return 0;
}

[[deprecated("discard function")]] double TemplaterMatcher::MatchTemplate(
    cv::Mat &img, cv::Mat &temp) {
  cv::Mat my_temp;
  cv::resize(temp, my_temp, img.size());
  int rows, cols;
  uchar *img_point, *temp_point;  // 像素类型uchar
  rows = my_temp.rows;
  cols = my_temp.cols * img.channels();
  double result, same = 0.0, different = 0.0;
  for (int i = 0; i < rows; i++) {  // 遍历图像像素
    // 获取像素值
    img_point = img.ptr<uchar>(i);
    temp_point = my_temp.ptr<uchar>(i);
    for (int j = 0; j < cols; j++) {
      if (img_point[j] == temp_point[j])
        same++;  // 记录像素相同的个数
      else
        different++;  // 记录像素不同的个数
    }
  }
  result = same / (same + different);
  SPDLOG_DEBUG("Armor Number Match Result:{}", result);
  return result;  // 返回匹配结果
}
