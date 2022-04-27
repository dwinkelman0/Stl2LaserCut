// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "LaserCut.h"

#include <iostream>
#include <numbers>
#include <set>

LaserCutRenderer::LaserCutRenderer(const Config &config) : config_(config) {}

static std::vector<RingVector<std::pair<BoundedLine, EdgePtr>>>
getNonIntersectingPolygons(
    const RingVector<std::pair<Vec2, std::vector<EdgePtr>>> &points) {
  // Generate edges
  std::vector<std::pair<Vec2, EdgePtr>> intersections;
  points.foreachPair([&points, &intersections](
                         const std::pair<Vec2, std::vector<EdgePtr>> &aMain,
                         const std::pair<Vec2, std::vector<EdgePtr>> &bMain) {
    const auto &[aPoint, aEdges] = aMain;
    const auto &[bPoint, bEdges] = bMain;
    BoundedLine mainLine(aPoint, bPoint);
    std::set<Vec2, Line::LinePointComparator> sortedIntersections(
        BoundedLine(aPoint, bPoint).getPointComparator());
    sortedIntersections.insert(aPoint);
    sortedIntersections.insert(bPoint);
    points.foreachPair([&mainLine, &sortedIntersections, &aMain, &bMain](
                           const std::pair<Vec2, std::vector<EdgePtr>> &a,
                           const std::pair<Vec2, std::vector<EdgePtr>> &b) {
      if (b.first != aMain.first && a.first != bMain.first) {
        std::optional<Vec2> intersection =
            mainLine.getBoundedIntersection(BoundedLine(a.first, b.first));
        if (intersection) {
          sortedIntersections.insert(*intersection);
        }
      }
    });
    EdgePtr edge = aEdges.back() == bEdges.front() ? aEdges.back() : nullptr;
    if (sortedIntersections.key_comp()(aPoint, bPoint)) {
      auto begin = sortedIntersections.find(aPoint);
      auto end = sortedIntersections.find(bPoint);
      for (auto it = begin; it != end; ++it) {
        intersections.emplace_back(*it, edge);
      }
    }
  });

  // Isolate non-intersecting polygons
  RingVector<std::pair<Vec2, EdgePtr>> mainPolygon(intersections);
  std::vector<RingVector<std::pair<Vec2, EdgePtr>>> subPolygons =
      mainPolygon.splitCycles([](const std::pair<Vec2, EdgePtr> &a,
                                 const std::pair<Vec2, EdgePtr> &b) {
        return abs(a.first - b.first) < 1e-6;
      });
  std::vector<RingVector<std::pair<BoundedLine, EdgePtr>>> output;
  for (const auto &ring : subPolygons) {
    Polygon polygon(ring.foreach<Vec2>(
        [](const std::pair<Vec2, EdgePtr> &a) { return a.first; }));
    if (polygon.getHandedness() == Polygon::Handedness::RIGHT) {
      output.push_back(ring.foreachPair<std::pair<BoundedLine, EdgePtr>>(
          [](const std::pair<Vec2, EdgePtr> &a,
             const std::pair<Vec2, EdgePtr> &b) {
            return std::pair<BoundedLine, EdgePtr>(
                BoundedLine(a.first, b.first), a.second);
          }));
    }
  }
  return output;
}

std::vector<std::vector<RenderedEdge>> LaserCutRenderer::renderFace(
    const FacePtr &face) const {
  // Project polygon and associate edges
  Polygon projectedPolygon(face);
  auto projectedEdges = zip(face->getEdges(), projectedPolygon.getLines());

  // Create perpendicular lines
  auto perpendicularLines = projectedEdges.foreachPair<std::optional<Line>>(
      [this](const std::pair<EdgePtr, BoundedLine> &a,
             const std::pair<EdgePtr, BoundedLine> &b) {
        const auto &[aEdge, aLine] = a;
        const auto &[bEdge, bLine] = b;
        if (abs(aLine.getAngle(bLine)) < std::numbers::pi / 4) {
          assert(aLine.getUpperBound() == bLine.getLowerBound());
          float aOffset = getGeometricEdgeDistance(aEdge->getAngle());
          float bOffset = getGeometricEdgeDistance(bEdge->getAngle());
          if (aOffset == bOffset) {
            return std::optional<Line>();
          } else {
            return std::optional<Line>(
                (aOffset > bOffset ? aLine : bLine)
                    .getPerpendicularLine(aLine.getUpperBound()));
          }
        } else {
          return std::optional<Line>();
        }
      });

  // Create polygon
  std::vector<std::pair<Vec2, std::vector<EdgePtr>>> baselineEdges;
  projectedEdges.sandwich<std::optional<Line>>(
      perpendicularLines,
      [this, &baselineEdges](const std::pair<EdgePtr, BoundedLine> &a,
                             const std::optional<Line> &perpendicularLine,
                             const std::pair<EdgePtr, BoundedLine> &b) {
        const auto &[aEdge, aLine] = a;
        const auto &[bEdge, bLine] = b;
        Line aOffsetLine =
            aLine.getOffsetLine(getGeometricEdgeDistance(aEdge->getAngle()));
        Line bOffsetLine =
            bLine.getOffsetLine(getGeometricEdgeDistance(bEdge->getAngle()));
        if (perpendicularLine) {
          std::optional<Vec2> aIntersection =
              aOffsetLine.getIntersection(*perpendicularLine);
          assert(aIntersection);
          std::optional<Vec2> bIntersection =
              bOffsetLine.getIntersection(*perpendicularLine);
          assert(bIntersection);
          baselineEdges.emplace_back(*aIntersection,
                                     std::vector<EdgePtr>({aEdge}));
          baselineEdges.emplace_back(*bIntersection,
                                     std::vector<EdgePtr>({bEdge}));
        } else {
          std::optional<Vec2> intersection =
              aOffsetLine.getIntersection(bOffsetLine);
          assert(intersection);
          baselineEdges.emplace_back(*intersection,
                                     std::vector<EdgePtr>({aEdge, bEdge}));
        }
      });
  projectedEdges.foreach ([](const std::pair<EdgePtr, BoundedLine> &a) {
    std::cout << a.first << ": " << a.second << std::endl;
  });

  auto polygons = getNonIntersectingPolygons(baselineEdges);
  for (const auto &polygon : polygons) {
    std::cout << "Polygon (" << polygon.getSize() << "):" << std::endl;
    polygon.foreach ([](const std::pair<BoundedLine, EdgePtr> &a) {
      std::cout << " - " << a.second << ": " << a.first << std::endl;
    });
  }

  return {};
}

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
