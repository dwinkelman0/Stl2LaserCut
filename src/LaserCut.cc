// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "LaserCut.h"

#include <DirectedGraph.h>
#include <Lookup.h>

#include <iostream>
#include <numbers>
#include <set>

LaserCutRenderer::LaserCutRenderer(const Config &config) : config_(config) {}

template <typename T>
class IntersectionComparator {
 public:
  IntersectionComparator(const Line &line)
      : comparator_(line.getPointComparator()) {}

  bool operator()(const std::pair<Vec2, T> &a,
                  const std::pair<Vec2, T> &b) const {
    return comparator_(a.first, b.first);
  }

 private:
  Line::LinePointComparator comparator_;
};

template <typename T>
static std::vector<RingVector<std::pair<BoundedLine, T>>>
getNonIntersectingPolygons(
    const RingVector<std::pair<BoundedLine, T>> &originalLines) {
  using IntersectionSet =
      std::set<std::pair<Vec2, uint32_t>, IntersectionComparator<uint32_t>>;
  RingVector<IntersectionSet> intersections =
      originalLines.template foreach<IntersectionSet>(
          [&intersections](const std::pair<BoundedLine, T> &item) {
            return IntersectionSet(item.first);
          });

  // Calculate and sort all intersections
  uint32_t outerCounter = 0;
  originalLines.foreach ([&outerCounter, &intersections,
                          &originalLines](const std::pair<BoundedLine, T> &a) {
    BoundedLine aLine = a.first;
    intersections[outerCounter].emplace(
        aLine.getUpperBound(), (outerCounter + 1) % intersections.getSize());
    intersections[outerCounter + 1].emplace(aLine.getUpperBound(),
                                            outerCounter);
    uint32_t innerCounter = 0;
    originalLines.foreach (
        [outerCounter, &innerCounter, &aLine,
         &intersections](const std::pair<BoundedLine, T> &b) {
          BoundedLine bLine = b.first;
          if (innerCounter < outerCounter &&
              aLine.getUpperBound() != bLine.getLowerBound() &&
              aLine.getLowerBound() != bLine.getUpperBound()) {
            std::optional<Vec2> intersection = aLine.getIntersection(bLine);
            if (intersection) {
              intersections[innerCounter].emplace(*intersection, outerCounter);
              intersections[outerCounter].emplace(*intersection, innerCounter);
            }
          }
          ++innerCounter;
        });
    ++outerCounter;
  });

  intersections.foreach ([](const IntersectionSet &s) {
    for (const auto &[point, index] : s) {
      std::cout << point << ", " << index << "; ";
    }
    std::cout << std::endl;
  });

  // Create a graph and filter out nodes not included in any cycles
  algo::DirectedGraph graph;
  algo::Lookup<Vec2> lookup;
  intersections.foreach ([&graph, &lookup](const IntersectionSet &set) {
    auto it2 = set.begin();
    for (auto it1 = it2++; it2 != set.end(); ++it1, ++it2) {
      graph.connect(lookup(it1->first), lookup(it2->first));
    }
  });
  algo::DirectedGraph prunedGraph = graph.pruneSourcesAndSinks();
  std::set<uint32_t> validPoints = prunedGraph.getNodes();

  // Create a sequence of vertices
  std::vector<std::pair<Vec2, uint32_t>> vertices;
  uint32_t counter = 0;
  intersections.foreach (
      [&vertices, &counter, &intersections](const IntersectionSet &set) {
        uint32_t startIndex =
            (counter + intersections.getSize() - 1) % intersections.getSize();
        uint32_t endIndex =
            (counter + intersections.getSize() + 1) % intersections.getSize();
        bool validZone = false;
        for (const auto &[point, index] : set) {
          if (index == startIndex) {
            validZone = true;
          }
          if (validZone) {
            if (index == endIndex) {
              break;
            }
            vertices.emplace_back(point, counter);
          }
        }
        ++counter;
      });

  std::cout << graph.getNodes().size() << " nodes, " << graph.getEdges().size()
            << " edges" << std::endl;
  std::cout << prunedGraph.getNodes().size() << " nodes, "
            << prunedGraph.getEdges().size() << " edges" << std::endl;
  std::cout << vertices.size() << " vertices" << std::endl;

  // Get distinct polygons
  std::vector<RingVector<std::pair<Vec2, uint32_t>>> polygons =
      RingVector(vertices).splitCycles([](const std::pair<Vec2, uint32_t> &a,
                                          const std::pair<Vec2, uint32_t> &b) {
        return a.first == b.first;
      });
  std::vector<RingVector<std::pair<BoundedLine, T>>> output;
  for (const auto &ring : polygons) {
    if (ring.getSize() >= 3) {
      Polygon polygon(ring);
      if (polygon.getHandedness() == Polygon::Handedness::RIGHT) {
        std::optional<std::pair<Vec2, uint32_t>> lastVertex;
        output.push_back(ring.foreachPair<std::pair<BoundedLine, T>>(
            [&lastVertex, &originalLines](const std::pair<Vec2, uint32_t> &a,
                                          const std::pair<Vec2, uint32_t> &b) {
              if (a.second == b.second) {
                if (!lastVertex) {
                  lastVertex = a;
                }
                return std::optional<std::pair<BoundedLine, T>>(std::nullopt);
              } else {
                auto output = std::make_optional<std::pair<BoundedLine, T>>(
                    BoundedLine((lastVertex ? lastVertex->first : a.first),
                                b.first),
                    originalLines[lastVertex ? lastVertex->second : a.second]
                        .second);
                lastVertex.reset();
                return output;
              }
            }));
      } else {
        std::cout << "Left-handed polygon" << std::endl;
      }
    } else {
      std::cout << "Invalid fragment" << std::endl;
    }
  }
  for (const auto &polygon : output) {
    std::cout << polygon.getSize() << " polygon" << std::endl;
  }

  return output;
}

std::vector<RingVector<std::pair<BoundedLine, uint32_t>>>
getNonIntersectingPolygonsTester(
    const RingVector<std::pair<BoundedLine, uint32_t>> &originalLines) {
  return getNonIntersectingPolygons<uint32_t>(originalLines);
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
          float aOffset = f1(aEdge->getAngle());
          float bOffset = f1(bEdge->getAngle());
          if (aOffset == bOffset) {
            return std::optional<Line>();
          } else {
            std::cout << "Perpendicular" << std::endl;
            return std::optional<Line>(
                (aOffset > bOffset ? aLine : bLine)
                    .getPerpendicularLine(aLine.getUpperBound()));
          }
        } else {
          return std::optional<Line>();
        }
      });

  // Create polygon
  std::vector<std::pair<Vec2, std::pair<EdgePtr, EdgePtr>>> baselineEdgePoints;
  projectedEdges.sandwich<std::optional<Line>>(
      perpendicularLines,
      [this, &baselineEdgePoints](const std::pair<EdgePtr, BoundedLine> &a,
                                  const std::optional<Line> &perpendicularLine,
                                  const std::pair<EdgePtr, BoundedLine> &b) {
        const auto &[aEdge, aLine] = a;
        const auto &[bEdge, bLine] = b;
        Line aOffsetLine = aLine.getOffsetLine(f1(aEdge->getAngle()));
        Line bOffsetLine = bLine.getOffsetLine(f1(bEdge->getAngle()));
        if (perpendicularLine) {
          std::optional<Vec2> aIntersection =
              aOffsetLine.getIntersection(*perpendicularLine);
          assert(aIntersection);
          std::optional<Vec2> bIntersection =
              bOffsetLine.getIntersection(*perpendicularLine);
          assert(bIntersection);
          baselineEdgePoints.emplace_back(
              *aIntersection, std::pair<EdgePtr, EdgePtr>(aEdge, nullptr));
          baselineEdgePoints.emplace_back(
              *bIntersection, std::pair<EdgePtr, EdgePtr>(nullptr, bEdge));
        } else {
          std::optional<Vec2> intersection =
              aOffsetLine.getIntersection(bOffsetLine);
          assert(intersection);
          baselineEdgePoints.emplace_back(
              *intersection, std::pair<EdgePtr, EdgePtr>(aEdge, bEdge));
        }
      });

  RingVector<std::pair<BoundedLine, EdgePtr>> baselineEdges =
      RingVector<std::pair<Vec2, std::pair<EdgePtr, EdgePtr>>>(
          baselineEdgePoints)
          .foreachPair<std::pair<BoundedLine, EdgePtr>>(
              [](const std::pair<Vec2, std::pair<EdgePtr, EdgePtr>> &a,
                 const std::pair<Vec2, std::pair<EdgePtr, EdgePtr>> &b) {
                assert(a.second.second == b.second.first);
                return a.first != b.first
                           ? std::make_optional<
                                 std::pair<BoundedLine, EdgePtr>>(
                                 BoundedLine(a.first, b.first), a.second.second)
                           : std::optional<std::pair<BoundedLine, EdgePtr>>(
                                 std::nullopt);
              });
  getNonIntersectingPolygons(baselineEdges);

  return {};
}

float LaserCutRenderer::f1(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  if (angle > 0) {
    return config_.materialThickness / std::tan((std::numbers::pi - angle) / 2);
  } else {
    return 0;
  }
}

float LaserCutRenderer::f2(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  if (abs(angle) < std::numbers::pi / 2) {
    return config_.materialThickness *
           (std::cos(abs(angle) - std::numbers::pi / 4) / std::sqrt(2) -
            1 / std::tan((std::numbers::pi - abs(angle)) / 2) / 2);
  } else {
    return 0;
  }
}

float LaserCutRenderer::f3(const float angle) const {
  assert(abs(angle) < std::numbers::pi);
  if (abs(angle) <= std::numbers::pi / 2) {
    return config_.materialThickness *
           (std::cos(abs(angle) / 2 - std::numbers::pi / 4) + 0.5);
  } else {
    return f1(abs(angle)) / 2 + config_.materialThickness / 2;
  }
}
