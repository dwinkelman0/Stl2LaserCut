// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Geometry.h"

#include <LaserCut.h>

#include <numbers>

Vec3 operator-(const Vec3 vec) {
  return {-std::get<0>(vec), -std::get<1>(vec), -std::get<2>(vec)};
}

Vec2 operator+(const Vec2 vec1, const Vec2 vec2) {
  return {std::get<0>(vec1) + std::get<0>(vec2),
          std::get<1>(vec1) + std::get<1>(vec2)};
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

Vec2 operator/(const Vec2 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x};
}

Vec3 operator/(const Vec3 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x, std::get<2>(vec) / x};
}

float dot(const Vec2 vec1, const Vec2 vec2) {
  return std::get<0>(vec1) * std::get<0>(vec2) +
         std::get<1>(vec1) * std::get<1>(vec2);
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

float abs(const Vec2 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec));
}

float abs(const Vec3 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec) +
                   std::get<2>(vec) * std::get<2>(vec));
}

float angle(const Vec2 vec1, const Vec2 vec2) {
  float mag = abs(vec1) * abs(vec2);
  return mag > 0 ? std::acos(dot(vec1, vec2) / mag) : 0;
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
  auto [it1, success1] =
      edges_.emplace(other, Edge::create(this->shared_from_this(), other));
  auto [it2, success2] = other->edges_.emplace(
      this->shared_from_this(), Edge::create(other, this->shared_from_this()));
  assert(success1 == success2);
  it1->second->leftFace_ = face;
  it2->second->rightFace_ = face;
}

EdgePtr Vertex::getEdge(const VertexPtr other) const {
  auto it = edges_.find(other);
  return it != edges_.end() ? it->second : nullptr;
}

float Edge::getAngle() const {
  assert(leftFace_);
  assert(rightFace_);
  return angle(leftFace_->getNormal(), rightFace_->getNormal()) *
         (isConvex() ? 1 : -1);
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
  vertices_.foreachPair([this](const VertexPtr &a, const VertexPtr &b) {
    a->link(shared_from_this(), b);
  });
}

RingVector<EdgePtr> Face::getEdges() const {
  return vertices_.foreachPair<EdgePtr>(
      [](const VertexPtr &a, const VertexPtr &b) { return a->getEdge(b); });
}

bool Face::isPlanar() const {
  bool output = true;
  vertices_.foreachPair(
      [this, &output](const VertexPtr &a, const VertexPtr &b) {
        if (a != vertices_[0] && b != vertices_[0]) {
          Vec3 basisEdge = a->getVector() - vertices_[0]->getVector();
          Vec3 secondEdge = b->getVector() - vertices_[0]->getVector();
          Vec3 product = cross(basisEdge, secondEdge);
          if (abs(product) > 0) {
            product = product / abs(product);
            float alignment = abs(dot(product, normal_));
            if (abs(alignment - 1) > 1e-6) {
              output = false;
            }
          } else {
            output = false;
          }
        }
      });
  return output;
}

float Face::getArea() const {
  float output = 0;
  vertices_.foreachPair(
      [this, &output](const VertexPtr &a, const VertexPtr &b) {
        if (a != vertices_[0] && b != vertices_[0]) {
          Vec3 basisEdge = a->getVector() - vertices_[0]->getVector();
          Vec3 secondEdge = b->getVector() - vertices_[0]->getVector();
          Vec3 product = cross(basisEdge, secondEdge);
          if (dot(product, normal_) > 0) {
            output += abs(product) / 2;
          } else {
            output -= abs(product) / 2;
          }
        }
      });
  return output;
}

std::optional<Vec2> Line::getIntersection(const Line &other) const {
  if (abs(a_ * other.b_ - b_ * other.a_) < 1e-6) {
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
  return Line(a_, b_, c_ + offset * sqrt(a_ * a_ + b_ * b_));
}

bool Line::getPossibleEquality(const Line &other) const {
  Line thisNorm = normalize();
  Line otherNorm = other.normalize();
  return abs(thisNorm.a_ - otherNorm.a_) < 1e-6 &&
         abs(thisNorm.b_ - otherNorm.b_) < 1e-6 &&
         abs(thisNorm.c_ - otherNorm.c_) < 1e-6;
}

Line Line::getPerpendicularLine(const Vec2 &point) const {
  return Line(b_, -a_, b_ * std::get<0>(point) - a_ * std::get<1>(point));
}

bool Line::comparePoints(const Vec2 &a, const Vec2 &b) const {
  std::optional<Vec2> aIntersect = getIntersection(getPerpendicularLine(a));
  std::optional<Vec2> bIntersect = getIntersection(getPerpendicularLine(b));
  assert(aIntersect && bIntersect);
  return dot(*bIntersect - *aIntersect, {b_, -a_}) > 0;
}

Line Line::normalize() const {
  float magnitude = std::sqrt(a_ * a_ + b_ * b_);
  return Line(a_ / magnitude, b_ / magnitude, c_ / magnitude);
}

Line Line::getMidline(const Line &other, const Vec2 &point) const {
  Line thisNorm = normalize();
  Line otherNorm = other.normalize();
  float newA = thisNorm.a_ + otherNorm.a_;
  float newB = thisNorm.b_ + otherNorm.b_;
  return Line(newA, newB,
              newA * std::get<0>(point) + newB * std::get<1>(point));
}

float Line::getAngle(const Line &other) const {
  float output = angle({a_, b_}, {other.a_, other.b_});
  return cross({a_, b_}, {other.a_, other.b_}) > 0 ? output : -output;
}

std::ostream &operator<<(std::ostream &os, const BoundedLine &line) {
  os << dynamic_cast<const Line &>(line) << ", ";
  if (line.b_ == 0) {
    os << std::min(std::get<1>(line.lowerBound_), std::get<1>(line.upperBound_))
       << " < y < "
       << std::max(std::get<1>(line.lowerBound_),
                   std::get<1>(line.upperBound_));
  } else {
    os << std::min(std::get<0>(line.lowerBound_), std::get<0>(line.upperBound_))
       << " < x < "
       << std::max(std::get<0>(line.lowerBound_),
                   std::get<0>(line.upperBound_));
  }
  return os;
}

BoundedLine::BoundedLine(const Vec2 b1, const Vec2 b2) : Line(0, 0, 0) {
  assert(b1 != b2);
  lowerBound_ = b1;
  upperBound_ = b2;
  b_ = std::get<0>(b2) - std::get<0>(b1);
  a_ = std::get<1>(b1) - std::get<1>(b2);
  c_ = a_ * std::get<0>(b1) + b_ * std::get<1>(b1);
}

std::optional<Vec2> BoundedLine::getBoundedIntersection(
    const BoundedLine &other) const {
  auto result = getIntersection(other);
  if (result && !comparePoints(*result, lowerBound_) &&
      !comparePoints(upperBound_, *result) &&
      !other.comparePoints(*result, other.lowerBound_) &&
      !other.comparePoints(other.upperBound_, *result)) {
    return result;
  } else {
    return std::nullopt;
  }
}

Vec2 BoundedLine::getMidpoint() const {
  return {(std::get<0>(upperBound_) + std::get<0>(lowerBound_)) / 2,
          (std::get<1>(upperBound_) + std::get<1>(lowerBound_)) / 2};
}

std::ostream &operator<<(std::ostream &os, const Line &line) {
  os << line.a_ << "x + " << line.b_ << "y = " << line.c_;
  return os;
}

Polygon::Polygon(const FacePtr &face) : points_({}) {
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
  points_ = face->getVertices().template foreach<Vec2>(
      [angleX, angleZ](const VertexPtr &vertex) {
        Vec3 transformed =
            rotateX(rotateZ(vertex->getVector(), angleZ), angleX);
        return Vec2(std::get<0>(transformed), std::get<1>(transformed));
      });
  points_.foreachPair([](const Vec2 &a, const Vec2 &b) { assert(a != b); });
}

bool Polygon::isSelfIntersecting() const {
  RingVector<BoundedLine> lines = getLines();
  for (uint32_t i = 0; i < lines.getSize(); ++i) {
    for (uint32_t j = i + 2;
         j < std::min(lines.getSize(), lines.getSize() - 1 + i); ++j) {
      if (lines[i].getBoundedIntersection(lines[j])) {
        return true;
      }
    }
  }
  return false;
}

float Polygon::getArea() const {
  float output = 0;
  points_.foreachPair([this, &output](const Vec2 &a, const Vec2 &b) {
    Vec2 basisEdge = a - points_[0];
    Vec2 secondEdge = b - points_[0];
    output += cross(basisEdge, secondEdge) / 2;
  });
  return output;
}

static Vec2 getOffsetPoint(const Line &a, const Line &b, const Vec2 point) {
  std::optional<Vec2> intersection = a.getIntersection(b);
  if (intersection) {
    return *intersection;
  } else {
    Line perpendicular = a.getPerpendicularLine(point);
    std::optional<Vec2> perpendicularIntersection =
        perpendicular.getIntersection(a);
    assert(perpendicularIntersection);
    return *perpendicularIntersection;
  }
}

Polygon::Handedness Polygon::getHandedness() const {
  float sum = 0;
  points_.foreachPair([&sum](const Vec2 &a, const Vec2 &b) {
    sum +=
        (std::get<0>(b) - std::get<0>(a)) * (std::get<1>(b) + std::get<1>(a));
  });
  return sum < 0 ? Handedness::RIGHT : Handedness::LEFT;
}

RingVector<BoundedLine> Polygon::getLines() const {
  return points_.foreachPair<BoundedLine>(
      [](const Vec2 &a, const Vec2 &b) { return BoundedLine(a, b); });
}

std::ostream &operator<<(std::ostream &os, const Polygon &polygon) {
  os << "{ ";
  polygon.getPoints().foreach (
      [&os](const Vec2 &point) { os << point << ", "; });
  os << "}";
  return os;
}

std::vector<EdgePtr> collectEdges(const std::vector<FacePtr> &faces) {
  std::vector<EdgePtr> edges;
  std::set<std::pair<VertexPtr, VertexPtr>> pairSets;
  for (const FacePtr &face : faces) {
    face->getVertices().foreach ([&edges, &pairSets](const VertexPtr &vertex) {
      for (const auto &[other, edge] : vertex->edges_) {
        const auto [v1, v2] = edge->getVertices();
        if (pairSets.find({v2, v1}) == pairSets.end() &&
            pairSets.find({v1, v2}) == pairSets.end()) {
          pairSets.insert({v1, v2});
          pairSets.insert({v2, v1});
          edges.push_back(edge);
        }
      }
    });
  }
  return edges;
}
