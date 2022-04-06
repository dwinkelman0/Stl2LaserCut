// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Geometry.h"

#include <numbers>

Vec3 operator-(const Vec3 vec) {
  return {-std::get<0>(vec), -std::get<1>(vec), -std::get<2>(vec)};
}

Vec2 operator-(const Vec2 vec1, const Vec2 vec2) {
  return {std::get<0>(vec1) - std::get<0>(vec2),
          std::get<1>(vec1) - std::get<1>(vec2)};
}

Vec3 operator-(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<0>(vec1) - std::get<0>(vec2),
          std::get<1>(vec1) - std::get<1>(vec2),
          std::get<2>(vec1) - std::get<2>(vec2)};
}

Vec3 operator/(const Vec3 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x, std::get<2>(vec) / x};
}

float dot(const Vec3 vec1, const Vec3 vec2) {
  return std::get<0>(vec1) * std::get<0>(vec2) +
         std::get<1>(vec1) * std::get<1>(vec2) +
         std::get<2>(vec1) * std::get<2>(vec2);
}

float cross(const Vec2 vec1, const Vec2 vec2) {
  return std::get<0>(vec1) * std::get<1>(vec2) -
         std::get<0>(vec2) * std::get<1>(vec1);
}

Vec3 cross(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<1>(vec1) * std::get<2>(vec2) -
              std::get<2>(vec1) * std::get<1>(vec2),
          std::get<2>(vec1) * std::get<0>(vec2) -
              std::get<0>(vec1) * std::get<2>(vec2),
          std::get<0>(vec1) * std::get<1>(vec2) -
              std::get<1>(vec1) * std::get<0>(vec2)};
}

float abs(const Vec3 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec) +
                   std::get<2>(vec) * std::get<2>(vec));
}

float angle(const Vec3 vec1, const Vec3 vec2) {
  float mag = abs(vec1) * abs(vec2);
  return mag > 0 ? std::acos(dot(vec1, vec2) / mag) : 0;
}

Vec3 rotateX(const Vec3 vec, const float angle) {
  return {
      std::get<0>(vec),
      std::cos(angle) * std::get<1>(vec) - std::sin(angle) * std::get<2>(vec),
      std::sin(angle) * std::get<1>(vec) + std::cos(angle) * std::get<2>(vec)};
}

Vec3 rotateY(const Vec3 vec, const float angle) {
  return {
      std::sin(angle) * std::get<2>(vec) + std::cos(angle) * std::get<0>(vec),
      std::get<1>(vec),
      std::cos(angle) * std::get<2>(vec) - std::sin(angle) * std::get<0>(vec)};
}

Vec3 rotateZ(const Vec3 vec, const float angle) {
  return {
      std::cos(angle) * std::get<0>(vec) - std::sin(angle) * std::get<1>(vec),
      std::sin(angle) * std::get<0>(vec) + std::cos(angle) * std::get<1>(vec),
      std::get<2>(vec)};
}

std::ostream &operator<<(std::ostream &os, const Vec2 &vec) {
  os << "(" << std::get<0>(vec) << ", " << std::get<1>(vec) << ")";
  return os;
}

std::ostream &operator<<(std::ostream &os, const Vec3 &vec) {
  os << "(" << std::get<0>(vec) << ", " << std::get<1>(vec) << ", "
     << std::get<2>(vec) << ")";
  return os;
}

void Vertex::link(const FacePtr &face, const VertexPtr &other) {
  auto newEdge = Edge::create(this->shared_from_this(), other);
  auto [it1, success1] = edges_.emplace(other, newEdge);
  auto [it2, success2] =
      other->edges_.emplace(this->shared_from_this(), newEdge);
  assert(success1 == success2);
  if (success1) {
    it1->second->leftFace_ = face;
  } else {
    it1->second->rightFace_ = face;
  }
}

float Edge::getAngle() const {
  assert(leftFace_);
  assert(rightFace_);
  return angle(leftFace_->getNormal(), rightFace_->getNormal());
}

bool Edge::isConvex() const {
  assert(leftFace_);
  assert(rightFace_);
  Vec3 vector = v2_->getVector() - v1_->getVector();
  Vec3 product = cross(leftFace_->getNormal(), rightFace_->getNormal());
  float alignment = dot(vector, product);
  return alignment > 0;
}

void Face::link() {
  for (uint32_t i = 0; i < vertices_.size() - 1; ++i) {
    vertices_[i]->link(this->shared_from_this(), vertices_[i + 1]);
  }
  vertices_.back()->link(this->shared_from_this(), vertices_.front());
}

bool Face::isPlanar() const {
  for (uint32_t i = 2; i < vertices_.size(); ++i) {
    Vec3 basisEdge = vertices_[i - 1]->getVector() - vertices_[0]->getVector();
    Vec3 secondEdge = vertices_[i]->getVector() - vertices_[0]->getVector();
    Vec3 product = cross(basisEdge, secondEdge);
    if (abs(product) > 0) {
      product = product / abs(product);
      float alignment = abs(dot(product, normal_));
      if (abs(alignment - 1) > 1e-6) {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

float Face::getArea() const {
  float output = 0;
  for (uint32_t i = 2; i < vertices_.size(); ++i) {
    Vec3 basisEdge = vertices_[i - 1]->getVector() - vertices_[0]->getVector();
    Vec3 secondEdge = vertices_[i]->getVector() - vertices_[0]->getVector();
    Vec3 product = cross(basisEdge, secondEdge);
    if (dot(product, normal_) > 0) {
      output += abs(product) / 2;
    } else {
      output -= abs(product) / 2;
    }
  }
  return output;
}

std::optional<Vec2> Line::getIntersection(const Line &other) const {
  if (a_ * other.b_ - b_ * other.a_ == 0) {
    return std::nullopt;
  } else if (a_ != 0) {
    float ratio = other.a_ / a_;
    float y = (other.c_ - ratio * c_) / (other.b_ - ratio * b_);
    float x = (c_ - b_ * y) / a_;
    return std::optional<Vec2>({x, y});
  } else {
    return other.getIntersection(*this);
  }
}

Line Line::getOffsetLine(const float offset) const {
  return Line(a_, b_,
              c_ + offset * sqrt(a_ * a_ + b_ * b_) * (direction_ ? 1 : -1),
              direction_);
}

bool Line::getPossibleEquality(const Line &other) const {
  return !getIntersection(other) &&
         ((b_ != 0 && other.b_ != 0 && c_ / b_ == other.c_ / other.b_) ||
          (a_ != 0 && other.a_ != 0 && c_ / a_ == other.c_ / other.a_));
}

std::ostream &operator<<(std::ostream &os, const BoundedLine &line) {
  os << dynamic_cast<const Line &>(line) << ", ";
  if (line.b_ == 0) {
    os << std::get<1>(line.lowerBound_) << " < y < "
       << std::get<1>(line.upperBound_);
  } else {
    os << std::get<0>(line.lowerBound_) << " < x < "
       << std::get<0>(line.upperBound_);
  }
  return os;
}

BoundedLine::BoundedLine(const Vec2 b1, const Vec2 b2) : Line(0, 0, 0, false) {
  if ((std::get<0>(b1) == std::get<0>(b2) &&
       std::get<1>(b1) < std::get<1>(b2)) ||
      std::get<0>(b1) < std::get<0>(b2)) {
    direction_ = true;
  } else {
    direction_ = false;
  }
  lowerBound_ = {std::min(std::get<0>(b1), std::get<0>(b2)),
                 std::min(std::get<1>(b1), std::get<1>(b2))};
  upperBound_ = {std::max(std::get<0>(b1), std::get<0>(b2)),
                 std::max(std::get<1>(b1), std::get<1>(b2))};
  auto result =
      Line(std::get<0>(b1), std::get<1>(b1), 1, true)
          .getIntersection(Line(std::get<0>(b2), std::get<1>(b2), 1, true));
  if (!result) {
    if (std::get<0>(b1) == 0 && std::get<0>(b2) == 0) {
      a_ = 1;
      b_ = 0;
    } else if (std::get<1>(b1) == 0 && std::get<1>(b2) == 0) {
      a_ = 0;
      b_ = 1;
    } else {
      a_ = std::get<1>(b2) - std::get<1>(b1);
      b_ = std::get<0>(b1) - std::get<0>(b2);
    }
    c_ = 0;
  } else {
    a_ = std::get<0>(*result);
    b_ = std::get<1>(*result);
    c_ = 1;
  }
  assert(std::get<0>(lowerBound_) <= std::get<0>(upperBound_));
  assert(std::get<1>(lowerBound_) <= std::get<1>(upperBound_));
}

std::optional<Vec2> BoundedLine::getBoundedIntersection(
    const BoundedLine &other) const {
  auto result = getIntersection(other);
  if (result) {
    float x = std::get<0>(*result);
    float y = std::get<1>(*result);
    if (std::get<0>(lowerBound_) <= x && x <= std::get<0>(upperBound_) &&
        std::get<1>(lowerBound_) <= y && y <= std::get<1>(upperBound_) &&
        std::get<0>(other.lowerBound_) <= x &&
        x <= std::get<0>(other.upperBound_) &&
        std::get<1>(other.lowerBound_) <= y &&
        y <= std::get<1>(other.upperBound_)) {
      return result;
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

std::ostream &operator<<(std::ostream &os, const Line &line) {
  os << line.a_ << "x + " << line.b_ << "y = " << line.c_;
  return os;
}

Polygon::Polygon(const FacePtr &face) {
  // Rotate about Z so that face is normal to an axis in YZ plane
  float angleZ = angle(cross(face->getNormal(), {0, 0, 1}), {1, 0, 0});
  Vec3 normal = rotateZ(face->getNormal(), angleZ);

  // Rotate about X so that face is normal to Z axis
  float angleX = angle(normal, {0, 0, 1});
  normal = rotateX(normal, angleX);

  // Get correct orientation
  float orientation = dot(normal, {0, 0, 1}) > 0 ? 0 : std::numbers::pi;
  angleX += orientation;

  // Process all points
  for (const VertexPtr &vertex : face->getVertices()) {
    Vec3 transformed = rotateX(rotateZ(vertex->getVector(), angleZ), angleX);
    points_.push_back({std::get<0>(transformed), std::get<1>(transformed)});
  }
}

bool Polygon::isSelfIntersecting() const {
  std::vector<BoundedLine> lines;
  for (uint32_t i = 0; i < points_.size() - 1; ++i) {
    lines.push_back(BoundedLine(points_[i], points_[i + 1]));
  }
  lines.push_back(BoundedLine(points_.back(), points_.front()));
  for (uint32_t i = 0; i < lines.size(); ++i) {
    for (uint32_t j = i + 2; j < std::min(lines.size(), lines.size() - 1 + i);
         ++j) {
      if (lines[i].getBoundedIntersection(lines[j])) {
        return true;
      }
    }
  }
  return false;
}

float Polygon::getArea() const {
  float output = 0;
  for (uint32_t i = 2; i < points_.size(); ++i) {
    Vec2 basisEdge = points_[i - 1] - points_[0];
    Vec2 secondEdge = points_[i] - points_[0];
    output += cross(basisEdge, secondEdge) / 2;
  }
  return output;
}

std::set<EdgePtr> collectEdges(const std::vector<FacePtr> &faces) {
  std::set<EdgePtr> edges;
  for (const FacePtr &face : faces) {
    for (const VertexPtr &vertex : face->vertices_) {
      for (const auto &[other, edge] : vertex->edges_) {
        edges.insert(edge);
      }
    }
  }
  return edges;
}
