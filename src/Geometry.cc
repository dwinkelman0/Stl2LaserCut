// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Geometry.h"

#include <LaserCut.h>
#include <Utils.h>

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

std::vector<EdgePtr> Face::getEdges() const {
  std::vector<EdgePtr> output;
  for (uint32_t i = 0; i < vertices_.size() - 1; ++i) {
    output.push_back(vertices_[i]->getEdge(vertices_[i + 1]));
  }
  output.push_back(vertices_.back()->getEdge(vertices_.front()));
  return output;
}

class LinePointComparator {
 public:
  LinePointComparator(const Line &line) : line_(line) {}
  bool operator()(const Vec2 &a, const Vec2 &b) const {
    return line_.comparePoints(a, b);
  }

 private:
  Line line_;
};

std::vector<std::vector<std::pair<Vec2, std::shared_ptr<Line>>>>
getNonIntersectingPolygons(const std::vector<std::shared_ptr<Line>> &mainLines,
                           const std::vector<Vec2> &intersections) {
  std::vector<std::tuple<std::shared_ptr<Line>, Vec2, Vec2,
                         std::set<Vec2, LinePointComparator>>>
      orderedIntersections;
  for (int32_t i = 0; i < mainLines.size(); ++i) {
    auto &[line, lowIntersection, highIntersection, intersectionSet] =
        orderedIntersections.emplace_back(mainLines[i], intersections[i],
                                          moduloIndex(intersections, i + 1),
                                          LinePointComparator(*mainLines[i]));
    intersectionSet.insert(lowIntersection);
    intersectionSet.insert(highIntersection);
  }
  for (uint32_t i = 0; i < orderedIntersections.size(); ++i) {
    for (uint32_t j = i + 2; j < std::min(orderedIntersections.size(),
                                          orderedIntersections.size() - 1 + i);
         ++j) {
      auto &[lineA, lowIntersectionA, highIntersectionA, intersectionSetA] =
          orderedIntersections[i];
      auto &[lineB, lowIntersectionB, highIntersectionB, intersectionSetB] =
          orderedIntersections[j];
      BoundedLine boundedLineA(lowIntersectionA, highIntersectionA);
      BoundedLine boundedLineB(lowIntersectionB, highIntersectionB);
      std::optional<Vec2> intersection =
          boundedLineA.getBoundedIntersection(boundedLineB);
      if (intersection) {
        intersectionSetA.insert(*intersection);
        intersectionSetB.insert(*intersection);
      }
    }
  }
  std::vector<std::vector<std::pair<Vec2, std::shared_ptr<Line>>>> output;
  std::vector<std::pair<Vec2, std::shared_ptr<Line>>> chain;
  for (const auto &[line, lowIntersection, highIntersection, intersectionSet] :
       orderedIntersections) {
    if (intersectionSet.key_comp()(lowIntersection, highIntersection)) {
      auto begin = intersectionSet.find(lowIntersection);
      auto end = intersectionSet.find(highIntersection);
      for (auto it = begin; it != end; ++it) {
        chain.emplace_back(*it, line);
      }
    }
  }
  while (!chain.empty()) {
    if (chain.size() < 3) {
      return output;
    }
    const auto [subchain, remainder] =
        getSmallestClosedShape<std::pair<Vec2, std::shared_ptr<Line>>>(
            chain, [](const std::pair<Vec2, std::shared_ptr<Line>> &a,
                      const std::pair<Vec2, std::shared_ptr<Line>> &b) {
              return a.first == b.first;
            });
    if (subchain.size() >= 3) {
      output.push_back(subchain);
    }
    chain = remainder;
  }
  return output;
}

void Face::generateBaselineEdges(const LaserCutRenderer &renderer) {
  std::vector<float> angles;
  for (const EdgePtr &edge : getEdges()) {
    angles.push_back(edge->getAngle());
  }
  std::vector<float> baselineOffsets;
  for (const float &angle : angles) {
    baselineOffsets.push_back(renderer.getMaterialBaseLength(angle));
    std::cout << baselineOffsets.back() << ", ";
  }
  std::cout << std::endl;
  Polygon projection(shared_from_this());
  std::vector<std::shared_ptr<Line>> partialOffsetLines;
  for (uint32_t i = 0; i < projection.getLines().size(); ++i) {
    partialOffsetLines.push_back(
        projection.getLines()[i]
            ? std::make_shared<Line>(
                  projection.getLines()[i]->getOffsetLine(-baselineOffsets[i]))
            : nullptr);
  }
  std::vector<std::shared_ptr<Line>> offsetLines, spacedOffsetLines;
  std::vector<Vec2> intersections;
  for (uint32_t i = 0; i < partialOffsetLines.size(); ++i) {
    Vec2 midpoint = moduloIndex(projection.getPoints(), i + 1);
    Line midline = partialOffsetLines[i]->getMidline(
        -*moduloIndex(partialOffsetLines, i + 1), midpoint);
    offsetLines.push_back(partialOffsetLines[i]);
    spacedOffsetLines.push_back(partialOffsetLines[i]);
    // TODO: fix midline insertion (perpendicularness, bounded offset)
    if (baselineOffsets[i] != moduloIndex(baselineOffsets, i + 1) &&
        abs(partialOffsetLines[i]->getAngle(
            *moduloIndex(partialOffsetLines, i + 1))) < std::numbers::pi / 4) {
      offsetLines.push_back(std::make_shared<BoundedLine>(
          *partialOffsetLines[i]->getIntersection(midline),
          *moduloIndex(partialOffsetLines, i + 1)->getIntersection(midline)));
      spacedOffsetLines.push_back(offsetLines.back());
      intersections.push_back(*partialOffsetLines[i]->getIntersection(midline));
      intersections.push_back(
          *moduloIndex(partialOffsetLines, i + 1)->getIntersection(midline));
    } else {
      spacedOffsetLines.push_back(nullptr);
      intersections.push_back(*partialOffsetLines[i]->getIntersection(midline));
    }
  }
  intersections.insert(intersections.begin(), intersections.back());
  intersections.pop_back();
  std::cout << offsetLines.size() << " offset lines" << std::endl;
  auto expandedPolygons =
      getNonIntersectingPolygons(offsetLines, intersections);

  std::cout << expandedPolygons.size() << " expanded polygons" << std::endl;
  std::cout << std::endl;
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
  return Line(a_, b_,
              c_ + offset * sqrt(a_ * a_ + b_ * b_) *
                       (b_ > 0 || b_ == 0 && a_ > 0 ? 1 : -1));
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
  return angle({a_, b_}, {other.a_, other.b_});
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
  std::vector<std::optional<BoundedLine>> lines = getLines();
  for (uint32_t i = 0; i < lines.size(); ++i) {
    for (uint32_t j = i + 2; j < std::min(lines.size(), lines.size() - 1 + i);
         ++j) {
      if (lines[i] && lines[j]) {
        if (lines[i]->getBoundedIntersection(*lines[j])) {
          return true;
        }
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
  for (uint32_t i = 0; i < points_.size() - 1; ++i) {
    sum += (std::get<0>(points_[i + 1]) - std::get<0>(points_[i])) *
           (std::get<1>(points_[i + 1]) + std::get<1>(points_[i]));
  }
  return sum < 0 ? Handedness::RIGHT : Handedness::LEFT;
}

std::vector<std::optional<BoundedLine>> Polygon::getLines() const {
  std::vector<std::optional<BoundedLine>> lines;
  for (uint32_t i = 0; i < points_.size() - 1; ++i) {
    lines.push_back(points_[i] != points_[i + 1]
                        ? std::optional<BoundedLine>(
                              BoundedLine(points_[i], points_[i + 1]))
                        : std::nullopt);
  }
  lines.push_back(BoundedLine(points_.back(), points_.front()));
  return lines;
}

std::ostream &operator<<(std::ostream &os, const Polygon &polygon) {
  os << "{ ";
  for (const Vec2 &point : polygon.getPoints()) {
    os << point << ", ";
  }
  os << "}";
  return os;
}

std::vector<EdgePtr> collectEdges(const std::vector<FacePtr> &faces) {
  std::vector<EdgePtr> edges;
  std::set<std::pair<VertexPtr, VertexPtr>> pairSets;
  for (const FacePtr &face : faces) {
    for (const VertexPtr &vertex : face->vertices_) {
      for (const auto &[other, edge] : vertex->edges_) {
        const auto [v1, v2] = edge->getVertices();
        if (pairSets.find({v2, v1}) == pairSets.end() &&
            pairSets.find({v1, v2}) == pairSets.end()) {
          pairSets.insert({v1, v2});
          pairSets.insert({v2, v1});
          edges.push_back(edge);
        }
      }
    }
  }
  return edges;
}
