#include "object.hpp"

#include "opencv2/gapi.hpp"

cv::gapi::wip::draw::Text draw::VisualizeLabel(const std::string &label,
                                               int level,
                                               const cv::Scalar &color) {
  int v_pos = 0;
  while (level < 0) level += 20;
  v_pos += 24 * level;
  return cv::gapi::wip::draw::Text(label, cv::Point2d(0, v_pos), draw::kCV_FONT,
                                   1.0, color);
}

const cv::Point2f &ImageObject::ImageCenter() const { return image_center_; }

std::vector<cv::Point2f> ImageObject::ImageVertices() const {
  return image_vertices_;
}

double ImageObject::ImageAngle() const { return image_angle_; }

double ImageObject::ImageAspectRatio() const { return image_ratio_; }

cv::Mat ImageObject::ImageFace(const cv::Mat &frame) const {
  cv::Mat face;
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

cv::gapi::wip::draw::Prims ImageObject::VisualizeObject(bool add_lable,
                                                        const cv::Scalar color,
                                                        cv::MarkerTypes type) {
  cv::gapi::wip::draw::Prims prims;
  auto vertices = ImageVertices();
  auto num_vertices = vertices.size();
  for (std::size_t i = 0; i < num_vertices; ++i)
    prims.emplace_back(cv::gapi::wip::draw::Line(
        vertices[i], vertices[(i + 1) % num_vertices], color));

  auto center = ImageCenter();
  if (type == cv::MarkerTypes::MARKER_CROSS) {
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x - draw::kMARKER, center.y),
        cv::Point(center.x + draw::kMARKER, center.y), color));
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x, center.y - draw::kMARKER),
        cv::Point(center.x, center.y + draw::kMARKER), color));
  } else if (type == cv::MarkerTypes::MARKER_DIAMOND) {
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x, center.y - draw::kMARKER),
        cv::Point(center.x + draw::kMARKER, center.y), color));
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x + draw::kMARKER, center.y),
        cv::Point(center.x, center.y + draw::kMARKER), color));
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x, center.y + draw::kMARKER),
        cv::Point(center.x - draw::kMARKER, center.y), color));
    prims.emplace_back(cv::gapi::wip::draw::Line(
        cv::Point(center.x - draw::kMARKER, center.y),
        cv::Point(center.x, center.y - draw::kMARKER), color));
  }

  if (add_lable) {
    prims.emplace_back(cv::gapi::wip::draw::Text(
        cv::format("%.2f, %.2f", ImageCenter().x, ImageCenter().y), vertices[1],
        draw::kCV_FONT, 1.0, color));
  }

  return prims;
}

const cv::Mat &PhysicObject::GetRotVec() const { return rot_vec_; }
void PhysicObject::SetRotVec(const cv::Mat &rot_vec) {
  rot_vec_ = rot_vec;
  cv::Rodrigues(rot_vec_, rot_mat_);
}

const cv::Mat &PhysicObject::GetRotMat() const { return rot_mat_; }
void PhysicObject::SetRotMat(const cv::Mat &rot_mat) {
  rot_mat_ = rot_mat;
  cv::Rodrigues(rot_mat_, rot_vec_);
}

const cv::Mat &PhysicObject::GetTransVec() const { return trans_vec_; }
void PhysicObject::SetTransVec(const cv::Mat &trans_vec) {
  trans_vec_ = trans_vec;
}

cv::Vec3d PhysicObject::RotationAxis() const {
  cv::Vec3d axis(rot_mat_.at<double>(2, 1) - rot_mat_.at<double>(1, 2),
                 rot_mat_.at<double>(0, 2) - rot_mat_.at<double>(2, 0),
                 rot_mat_.at<double>(1, 0) - rot_mat_.at<double>(0, 1));
  return axis;
}
const cv::Mat PhysicObject::PhysicVertices() const { return physic_vertices_; }
