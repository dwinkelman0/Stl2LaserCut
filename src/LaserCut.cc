// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "LaserCut.h"

#include <numbers>

std::vector<BoundedLine> getOverlappingBaselineEdges(
    const RenderedEdgePtr &edge,
    const std::vector<RenderedEdgePtr> &oppositeEdges) {
  return {};
}

static Vec2 projectPointToEdge(const Line &line, const Vec2 &origin,
                               const Vec2 &point) {
  std::optional<Vec2> intersection =
      line.getIntersection(line.getPerpendicularLine(point));
  assert(intersection);
  float direction =
      cross(*intersection - origin, point - *intersection) > 0 ? 1 : -1;
  return {abs(*intersection - origin) * direction, abs(point - *intersection)};
}

BoundedLine RenderedEdge::normalize() const {
  return BoundedLine(
      projectPointToEdge(geometricEdge_, geometricEdge_.getLowerBound(),
                         baselineEdge_.getLowerBound()),
      projectPointToEdge(geometricEdge_, geometricEdge_.getLowerBound(),
                         baselineEdge_.getUpperBound()));
}

LaserCutRenderer::LaserCutRenderer(const Config &config) : config_(config) {}

float LaserCutRenderer::getGeometricEdgeDistance(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  float output = 0;
  if (angle <= -std::numbers::pi / 2) {
    output = 0;
  } else if (angle <= 0) {
    output = getGeometricEdgeDistance(-angle) - std::tan(-angle / 2);
  } else if (angle <= std::numbers::pi / 2) {
    output = std::sin(angle + std::numbers::pi / 4) / std::sqrt(2) +
             std::tan(angle / 2) / 2;
  } else {
    output = 1 / std::tan((std::numbers::pi - angle) / 2);
  }
  return output * config_.materialThickness;
}

float LaserCutRenderer::getToothLength(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  float output = 0;
  if (abs(angle) <= std::numbers::pi / 2) {
    output = std::sin(abs(angle) + std::numbers::pi / 4) / std::sqrt(2) + 0.5;
  } else {
    output = (1 / std::tan((std::numbers::pi - abs(angle)) / 2) + 1) / 2;
  }
  return output * config_.materialThickness;
}

float LaserCutRenderer::getMaterialBaseLength(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  if (angle > 0) {
    return 1 / std::tan((std::numbers::pi - angle) / 2) *
           config_.materialThickness;
  } else {
    return 0;
  }
}
