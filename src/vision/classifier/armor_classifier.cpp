#include "armor_classifier.hpp"

#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

ArmorClassifier::ArmorClassifier(const std::string model_path,
                                 const std::string lable_path,
                                 const cv::Size &input_size) {
  LoadModel(model_path);
  LoadLable(lable_path);
  SetInputSize(input_size);
  SPDLOG_TRACE("Constructed.");
}

ArmorClassifier::ArmorClassifier() { SPDLOG_TRACE("Constructed."); }
ArmorClassifier::~ArmorClassifier() { SPDLOG_TRACE("Destructed."); }

void ArmorClassifier::LoadModel(const std::string &path) {
  net_ = cv::dnn::readNet(path);
}

void ArmorClassifier::LoadLable(const std::string &path) {
  cv::FileStorage fs(path,
                     cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);

  cv::FileNode root = fs.root();
  for (size_t i = 0; i < root.size(); ++i) {
    classes_.push_back(
        game::StringToModel(std::string(root[std::to_string(i)])));
  }
}

void ArmorClassifier::SetInputSize(const cv::Size &input_size) {
  net_input_size_ = input_size;
}

void ArmorClassifier::ClassifyModel(Armor &armor, const cv::Mat &frame) {
  cv::Mat image = armor.Face(frame);
  cv::dnn::blobFromImage(image, blob_, 1. / 128., net_input_size_);
  net_.setInput(blob_);
  cv::Mat prob = net_.forward();
  cv::Point class_point;
  cv::minMaxLoc(prob.reshape(1, 1), nullptr, &conf_, nullptr, &class_point);
  model_ = classes_[class_point.x];
  armor.SetModel(model_);
}
int ArmorClassifier::GetArmorId(cv::Mat &armor_face) {
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
                   cv::CHAIN_APPROX_NONE);  // 查找轮廓
  // cv::drawContours(clone_img, contours, -1, cv::Scalar(0, 255, 0), 1,
  //                  8);                               // 绘制轮廓
  std::vector<cv::Rect> sort_rect(contours.size());  // 外接矩形
  std::vector<cv::Mat> img_mat;  // 存储原图中的有效数字区域
  for (int i = 0; size_t(i) < contours.size(); i++) {
    sort_rect[i] = cv::boundingRect(contours[i]);  // 外接矩形
    cv::Mat roi = clone_img1(sort_rect[i]);
    cv::Mat dstroi;
    cv::resize(roi, dstroi, roi.size(), 0, 0);  // 重设大小
    img_mat.push_back(dstroi);
    // 绘制外接矩形
    // cv::rectangle(clone_img, cv::Point(sort_rect[i].x, sort_rect[i].y),
    //               cv::Point(sort_rect[i].x + sort_rect[i].width,
    //                         sort_rect[i].y + sort_rect[i].height),
    //               cv::Scalar(0, 0, 255), 1, 8);
  }
  // 对矩形进行排序，因为轮廓的顺序不一定是数字真正的顺序
  for (int i = 0; size_t(i) < sort_rect.size(); i++) {
    for (int j = i + 1; size_t(j) < sort_rect.size(); j++) {
      int j_x = sort_rect[j].x;
      int i_x = sort_rect[i].x;
      if (j_x < i_x) {
        cv::Mat temps = img_mat[i];
        img_mat[i] = img_mat[j];
        img_mat[j] = temps;
      }
    }
  }
  std::vector<cv::Mat> myTemplate;  // 模板容器
  for (int i = 1; i < 4; i++) {
    std::string name = cv::format("./template/%d.png", i);
    cv::Mat temp = cv::imread(name);
    if (temp.empty()) {
      // std::cout << "模板读取失败" << std::endl;
      SPDLOG_ERROR("模板读取失败");
      return -1;
    }
    cv::cvtColor(temp, temp, cv::COLOR_BGR2GRAY);
    cv::threshold(temp, temp, 100, 255, cv::THRESH_BINARY);
    myTemplate.push_back(temp);
  }
  if (myTemplate.empty()) {
    // std::cout << "模板导入失败" << std::endl;
    SPDLOG_ERROR("模板导入失败");
    return -2;
  }
  if (img_mat.empty()) {
    // std::cout << "二值化处理之后的源导入失败" << std::endl;
    SPDLOG_ERROR("二值化处理之后的源导入失败");
    return -2;
  }
  std::vector<int> seq;  // 顺序存放识别结果
  for (int i = 0; size_t(i) < img_mat.size(); i++) {
    double com = 0;
    double min = 0;
    int min_seq = 0;  // 记录识别结果
    for (int j = 0; size_t(j) < myTemplate.size(); j++) {
      // cv::imshow("s", img_mat[i]);
      // cv::imshow("t", myTemplate[j]);
      // cv::waitKey(5000);
      com = MatchTemplate(img_mat[i], myTemplate[j]);
      if (com > min) {
        min = com;
        min_seq = j;
      }
      com = 0;
    }
    seq.push_back(min_seq + 1);
  }
  // 输出结果
  //        std::cout << "识别结果为：";
  //        for (int i = 0; i < seq.size(); i++)
  //        std::cout << seq[i];
  if (!seq.empty()) return seq[0] + 1;
  return 0;
}
double ArmorClassifier::MatchTemplate(cv::Mat &img, cv::Mat &temp) {
  cv::Mat my_temp;
  cv::resize(temp, my_temp, img.size());
  int rows, cols;
  uchar *img_point, *temp_point;  // 像素类型uchar
  rows = my_temp.rows;
  cols = my_temp.cols * img.channels();
  double result, same = 0.0, different = 0.0;
  for (int i = 0; i < rows; i++)  // 遍历图像像素
  {
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