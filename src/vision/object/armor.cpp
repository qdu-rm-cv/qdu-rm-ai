#include "armor.hpp"

#include "opencv2/opencv.hpp"
#include "spdlog/spdlog.h"

namespace {

const double kARMOR_WIDTH = 125.;
const double kARMOR_LENGTH_SMALL = 135.;
const double kARMOR_LENGTH_BIG = 230.;
const double kARMOR_HEIGHT = kARMOR_WIDTH * std::sin(75. / 180. * M_PI);
const double kARMOR_DEPTH = kARMOR_WIDTH * std::cos(75. / 180. * M_PI);
const double kHIT_DEPTH = kARMOR_WIDTH / 2. * std::cos(75. / 180. * M_PI);

std::vector<cv::Point2f> kDST_POV_SMALL{
    cv::Point(0, kARMOR_WIDTH),
    cv::Point(0, 0),
    cv::Point(kARMOR_LENGTH_SMALL, 0),
    cv::Point(kARMOR_LENGTH_SMALL, kARMOR_WIDTH),
};

std::vector<cv::Point2f> kDST_POV_BIG{
    cv::Point(0, kARMOR_WIDTH),
    cv::Point(0, 0),
    cv::Point(kARMOR_LENGTH_BIG, 0),
    cv::Point(kARMOR_LENGTH_BIG, kARMOR_WIDTH),
};

/* clang-format off */
const cv::Matx43d kCOORD_SMALL_ARMOR(
    -kARMOR_LENGTH_SMALL / 2., kARMOR_HEIGHT / 2, -kARMOR_DEPTH / 2.,
    -kARMOR_LENGTH_SMALL / 2., -kARMOR_HEIGHT / 2, kARMOR_DEPTH / 2.,
    kARMOR_LENGTH_SMALL / 2., -kARMOR_HEIGHT / 2, kARMOR_DEPTH / 2.,
    kARMOR_LENGTH_SMALL / 2., kARMOR_HEIGHT / 2, -kARMOR_DEPTH / 2.);

const cv::Matx43d kCOORD_BIG_ARMOR(
    -kARMOR_LENGTH_BIG / 2., kARMOR_HEIGHT / 2, -kARMOR_DEPTH / 2.,
    -kARMOR_LENGTH_BIG / 2., -kARMOR_HEIGHT / 2, kARMOR_DEPTH / 2.,
    kARMOR_LENGTH_BIG / 2., -kARMOR_HEIGHT / 2, kARMOR_DEPTH / 2.,
    kARMOR_LENGTH_BIG / 2., kARMOR_HEIGHT / 2, -kARMOR_DEPTH / 2.);

const cv::Matx43d kCOORD_BUFF_ARMOR(
    -kARMOR_LENGTH_BIG / 2., -kARMOR_WIDTH / 2, 0.,
    kARMOR_LENGTH_BIG / 2., -kARMOR_WIDTH / 2, 0.,
    kARMOR_LENGTH_BIG / 2., kARMOR_WIDTH / 2, 0.,
    -kARMOR_LENGTH_BIG / 2., kARMOR_WIDTH / 2, 0.);
/* clang-format on */

const cv::Point3f kHIT_TARGET(0., 0., kHIT_DEPTH);

}  // namespace

cv::RotatedRect Armor::FormRect(const LightBar &left_bar,
                                const LightBar &right_bar) {
  const cv::Point2f center =
      (left_bar.ImageCenter() + right_bar.ImageCenter()) / 2.;
  const cv::Size size(
      cv::norm(left_bar.ImageCenter() - right_bar.ImageCenter()),
      (left_bar.Length() + right_bar.Length()));
  const float angle = (left_bar.ImageAngle() + right_bar.ImageAngle()) / 2.f;
  return cv::RotatedRect(center, size, angle);
}

void Armor::Init() {
  image_center_ = rect_.center;
  image_angle_ = rect_.angle;
  image_ratio_ = std::max(rect_.size.height, rect_.size.width) /
                 std::min(rect_.size.height, rect_.size.width);
  image_vertices_.resize(4);
  rect_.points(image_vertices_.data());
}

Armor::Armor() { SPDLOG_TRACE("Constructed."); }

Armor::Armor(const LightBar &left_bar, const LightBar &right_bar) {
  rect_ = FormRect(left_bar, right_bar);
  Init();
  SPDLOG_TRACE("Constructed.");
}

Armor::Armor(const cv::RotatedRect &rect) {
  rect_ = rect;
  Init();
  SPDLOG_TRACE("Constructed.");
}

Armor::~Armor() { SPDLOG_TRACE("Destructed."); }

game::Model Armor::GetModel() const { return model_; }
void Armor::SetModel(game::Model model) {
  model_ = model;

  if (model_ == game::Model::kBUFF) {
    physic_vertices_ = cv::Mat(kCOORD_BUFF_ARMOR);
  } else if (this->IsBigArmor()) {
    physic_vertices_ = cv::Mat(kCOORD_BIG_ARMOR);
  } else {
    physic_vertices_ = cv::Mat(kCOORD_SMALL_ARMOR);
  }
}

const cv::RotatedRect Armor::GetRect() const { return rect_; }

cv::Mat Armor::Face(const cv::Mat &frame) {
  double len;
  cv::Mat face;
  std::vector<cv::Point2f> pts(4);

  if (ImageAspectRatio() > 1.2) {
    trans_ = cv::getPerspectiveTransform(ImageVertices(), kDST_POV_BIG);
    len = kARMOR_LENGTH_BIG;
  } else {
    trans_ = cv::getPerspectiveTransform(ImageVertices(), kDST_POV_SMALL);
    len = kARMOR_LENGTH_SMALL;
  }
  face_size_ = cv::Size(len, kARMOR_WIDTH);

  cv::warpPerspective(frame, face, trans_, face_size_);

  cv::cvtColor(face, face, cv::COLOR_RGB2GRAY);
  cv::medianBlur(face, face, 1);
#if 0
  cv::equalizeHist(face, face); /* Tried. No help. */
#endif
  cv::threshold(face, face, 0., 255., cv::THRESH_BINARY | cv::THRESH_TRIANGLE);

  /* 截取中间正方形 */
  float min_edge = std::min(face.cols, face.rows);
  const int offset_w = (face.cols - min_edge) / 2;
  const int offset_h = (face.rows - min_edge) / 2;
  face = face(cv::Rect(offset_w, offset_h, min_edge, min_edge));
  return face;
}
double Armor::GetArea() { return rect_.size.width * rect_.size.height; }
component::Euler Armor::GetAimEuler() const { return aiming_euler_; }
void Armor::SetAimEuler(const component::Euler &elur) { aiming_euler_ = elur; }

bool Armor::IsBigArmor() {
  double light_length1 =
      cv::norm(this->image_vertices_[0] - this->image_vertices_[1]);
  double light_length2 =
      cv::norm(this->image_vertices_[4] - this->image_vertices_[3]);
  double aspect_ratio =
      this->GetRect().size.aspectRatio();  // double aspect_ratio = cv::norm();
  double heightScale = light_length1 > light_length2
                           ? (light_length1 / light_length2)
                           : (light_length2 / light_length1);
  if (aspect_ratio > 2.0) {
    return true;
  } else if (aspect_ratio > 1.5) {
    if (heightScale > 1.3) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}
