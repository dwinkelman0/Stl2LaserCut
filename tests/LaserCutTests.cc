// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <LaserCut.h>
#include <gtest/gtest.h>

std::vector<RingVector<std::pair<BoundedLine, uint32_t>>>
getNonIntersectingPolygonsTester(
    const RingVector<std::pair<BoundedLine, uint32_t>> &originalLines);

static RingVector<std::pair<BoundedLine, uint32_t>> getPointsRing(
    const RingVector<Vec2> &points) {
  uint32_t counter = 0;
  return points.template foreachPair<std::pair<BoundedLine, uint32_t>>(
      [&counter](const Vec2 &a, const Vec2 &b) {
        return std::pair<BoundedLine, uint32_t>(BoundedLine(a, b), counter++);
      });
}

TEST(LaserCut, NonIntersectingPolygons) {
  // Right-handed triangle
  auto p0 = getNonIntersectingPolygonsTester(
      getPointsRing(RingVector<Vec2>({{0, 0}, {1, 0}, {0, 1}})));
  ASSERT_EQ(p0.size(), 1);
  ASSERT_EQ(p0[0].getSize(), 3);
  ASSERT_EQ(p0[0][0].second, 0);
  ASSERT_EQ(p0[0][0].first.getLowerBound(), Vec2(0, 0));
  ASSERT_EQ(p0[0][0].first.getUpperBound(), Vec2(1, 0));

  // Left-handed triangle
  auto p1 = getNonIntersectingPolygonsTester(
      getPointsRing(RingVector<Vec2>({{0, 0}, {0, 1}, {1, 0}})));
  ASSERT_EQ(p1.size(), 0);

  // Self-intersecting quadrilateral
  auto p2 = getNonIntersectingPolygonsTester(
      getPointsRing(RingVector<Vec2>({{0, 0}, {1, 0}, {0, 1}, {1, 1}})));
  ASSERT_EQ(p2.size(), 1);
  ASSERT_EQ(p2[0].getSize(), 3);

  // Bowtie that creates two sub-triangles
  auto p3 = getNonIntersectingPolygonsTester(getPointsRing(
      RingVector<Vec2>({{0, 0}, {1, 1}, {2, 0}, {2, 2}, {1, 1}, {0, 2}})));
  ASSERT_EQ(p3.size(), 2);
  ASSERT_EQ(p3[0].getSize(), 3);
  ASSERT_EQ(p3[1].getSize(), 3);

  // Different kind of bowtie that creates two sub-triangles
  auto p4 = getNonIntersectingPolygonsTester(getPointsRing(
      RingVector<Vec2>({{0, 0}, {2, 0}, {2, 1}, {1, 0}, {0, 1}})));
  ASSERT_EQ(p4.size(), 2);
  ASSERT_EQ(p4[0].getSize(), 3);
  ASSERT_EQ(p4[1].getSize(), 3);
}
