// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Geometry.h>
#include <gtest/gtest.h>

TEST(Geometry, FaceIsPlanar) {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({3, 4, 0});
  VertexPtr v4 = Vertex::create({0, 1, 1});
  VertexPtr v5 = Vertex::create({-1, -1, 0});
  FacePtr f0 = Face::create(RingVector<VertexPtr>({v0, v1, v2}), {0, 0, 1});
  ASSERT_TRUE(f0);
  ASSERT_TRUE(f0->isPlanar());
  FacePtr f1 = Face::create(RingVector<VertexPtr>({v0, v1, v3, v2}), {0, 0, 1});
  ASSERT_TRUE(f1);
  ASSERT_TRUE(f1->isPlanar());
  FacePtr f2 = Face::create(RingVector<VertexPtr>({v5, v0, v1, v2}), {0, 0, 1});
  ASSERT_TRUE(f2);
  ASSERT_TRUE(f2->isPlanar());

  // Faces must be planar
  ASSERT_FALSE(Face::create(RingVector<VertexPtr>({v0, v1, v2}),
                            {0, 1 / std::sqrt(2), 1 / std::sqrt(2)}));
  ASSERT_FALSE(
      Face::create(RingVector<VertexPtr>({v0, v1, v2, v4}), {0, 0, 1}));
}

TEST(Geometry, FaceArea) {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({2, 1, 0});
  FacePtr f0 = Face::create(RingVector<VertexPtr>({v0, v1, v2}), {0, 0, 1});
  FacePtr f1 = Face::create(RingVector<VertexPtr>({v0, v3, v2}), {0, 0, 1});
  FacePtr f2 = Face::create(RingVector<VertexPtr>({v0, v1, v3, v2}), {0, 0, 1});
  ASSERT_FLOAT_EQ(f0->getArea(), 0.5);
  ASSERT_FLOAT_EQ(f1->getArea(), 1.0);
  ASSERT_FLOAT_EQ(f2->getArea(), 1.5);

  // Cannot create faces with negative area, indicates problem with handedness
  // of order of points
  ASSERT_FALSE(Face::create(RingVector<VertexPtr>({v0, v2, v3}), {0, 0, 1}));
}

TEST(Geometry, LineIntersection) {
  // Easy line
  Line l0(1, 1, 1);
  Line l1(1, -1, 0);
  auto i0 = l0.getIntersection(l1);
  ASSERT_TRUE(i0);
  ASSERT_FLOAT_EQ(std::get<0>(*i0), 0.5);
  ASSERT_FLOAT_EQ(std::get<1>(*i0), 0.5);

  // Trickier line
  Line l2(5, 3, 6);
  Line l3(8, -5, 9.6);
  auto i1 = l2.getIntersection(l3);
  ASSERT_TRUE(i1);
  ASSERT_FLOAT_EQ(std::get<0>(*i1), 1.2);
  ASSERT_FLOAT_EQ(std::get<1>(*i1), 0);

  // Infinitely many solutions
  Line l4(4, 2, 8);
  Line l5(12, 6, 24);
  auto i2 = l4.getIntersection(l5);
  ASSERT_FALSE(i2);

  // No solutions
  Line l6(4, 2, 10);
  Line l7(12, 6, 24);
  auto i3 = l6.getIntersection(l7);
  ASSERT_FALSE(i3);
}

TEST(Geometry, LineOffset) {
  // Easy line
  Line l0(1, 1, 0);
  Line c0(1, 1, std::sqrt(2));
  ASSERT_TRUE(l0.getOffsetLine(1).getPossibleEquality(c0));
  Line l1(-1, -1, 0);
  Line c1(-1, -1, -std::sqrt(2));
  ASSERT_TRUE(l1.getOffsetLine(1).getPossibleEquality(c1));

  // Vertical Line
  Line l2(1, 0, 0);
  Line c2(1, 0, 1);
  ASSERT_TRUE(l2.getOffsetLine(1).getPossibleEquality(c2));
  Line l3(-1, 0, 0);
  Line c3(-1, 0, -1);
  ASSERT_TRUE(l3.getOffsetLine(1).getPossibleEquality(c3));

  // Horizontal Line
  Line l4(0, 1, 0);
  Line c4(0, 1, 1);
  ASSERT_TRUE(l4.getOffsetLine(1).getPossibleEquality(c4));
  Line l5(0, -1, 0);
  Line c5(0, -1, -1);
  ASSERT_TRUE(l5.getOffsetLine(1).getPossibleEquality(c5));

  // Different signs
  Line l6(-1, -1, 1);
  Line c6(-1, -1, 1 - std::sqrt(2));
  ASSERT_TRUE(l6.getOffsetLine(1).getPossibleEquality(c6));
  Line l7(-1, 1, 1);
  Line c7(-1, 1, 2);
  ASSERT_TRUE(l7.getOffsetLine(std::sqrt(2) / 2).getPossibleEquality(c7));
  Line l8(1, -1, 1);
  Line c8(1, -1, 0);
  ASSERT_TRUE(l8.getOffsetLine(std::sqrt(2) / 2).getPossibleEquality(c8));
}

TEST(Geometry, LinePointComparison) {
  // Basic example (oriented towards left)
  BoundedLine l0({2, 0}, {0, 2});
  ASSERT_FALSE(l0.comparePoints({1, 1}, {2, 0}));
  ASSERT_FALSE(l0.comparePoints({1, 1}, {1, 1}));
  ASSERT_TRUE(l0.comparePoints({1, 1}, {0, 2}));
  ASSERT_FALSE(l0.comparePoints({0, 0}, {2, 2}));
  ASSERT_FALSE(l0.comparePoints({2, 2}, {0, 0}));
  ASSERT_TRUE(l0.comparePoints({2, 2}, {0, 2}));

  // Basic example (oriented towards right)
  BoundedLine l1({0, 2}, {2, 0});
  ASSERT_TRUE(l1.comparePoints({1, 1}, {2, 0}));
  ASSERT_FALSE(l1.comparePoints({1, 1}, {1, 1}));
  ASSERT_FALSE(l1.comparePoints({1, 1}, {0, 2}));
  ASSERT_FALSE(l1.comparePoints({0, 0}, {2, 2}));
  ASSERT_FALSE(l1.comparePoints({2, 2}, {0, 0}));
  ASSERT_FALSE(l1.comparePoints({2, 2}, {0, 2}));

  // Horizontal line (oriented towards left)
  BoundedLine l2({2, 0}, {0, 0});
  ASSERT_TRUE(l2.comparePoints({4, 0}, {3, 0}));
  ASSERT_FALSE(l2.comparePoints({3, 0}, {3, 0}));
  ASSERT_FALSE(l2.comparePoints({2, 0}, {3, 0}));

  // Horizontal line (oriented towards right)
  BoundedLine l3({0, 0}, {2, 0});
  ASSERT_FALSE(l3.comparePoints({4, 0}, {3, 0}));
  ASSERT_FALSE(l3.comparePoints({3, 0}, {3, 0}));
  ASSERT_TRUE(l3.comparePoints({2, 0}, {3, 0}));

  // Vertical line (oriented down)
  BoundedLine l4({0, 0}, {0, -3});
  ASSERT_TRUE(l4.comparePoints({0, 4}, {0, 3}));
  ASSERT_FALSE(l4.comparePoints({0, 3}, {0, 3}));
  ASSERT_FALSE(l4.comparePoints({0, 2}, {0, 3}));

  // Vertical line (oriented up)
  BoundedLine l5({0, -3}, {0, 0});
  ASSERT_FALSE(l5.comparePoints({0, 4}, {0, 3}));
  ASSERT_FALSE(l5.comparePoints({0, 3}, {0, 3}));
  ASSERT_TRUE(l5.comparePoints({0, 2}, {0, 3}));
}

TEST(Geometry, BoundedLine) {
  BoundedLine l0({2, 0}, {0, 2});
  Line c0(-1, -1, -2);
  ASSERT_TRUE(c0.getPossibleEquality(l0));

  BoundedLine l1({0, 2}, {2, 0});
  Line c1(1, 1, 2);
  ASSERT_TRUE(c1.getPossibleEquality(l1));

  BoundedLine l2({-2, -1}, {2, 1});
  Line c2(-1, 2, 0);
  ASSERT_TRUE(c2.getPossibleEquality(l2));

  BoundedLine l3({5, 0}, {5, 6});
  Line c3(-1, 0, -5);
  ASSERT_TRUE(c3.getPossibleEquality(l3));

  BoundedLine l4({0, 5}, {6, 5});
  Line c4(0, 1, 5);
  ASSERT_TRUE(c4.getPossibleEquality(l4));

  BoundedLine l5({0, 0}, {1, 0});
  Line c5(0, 1, 0);
  ASSERT_TRUE(c5.getPossibleEquality(l5));

  BoundedLine l6({0, 1}, {0, 0});
  Line c6(1, 0, 0);
  ASSERT_TRUE(c6.getPossibleEquality(l6));
}

TEST(Geometry, BoundedLineIntersection) {
  BoundedLine l0({2, 0}, {0, 2});
  BoundedLine l1({0, 0}, {1, 1});
  BoundedLine l2({-1, -1}, {0, 0});
  BoundedLine l3({3, 1}, {1, 3});

  ASSERT_TRUE(l0.getBoundedIntersection(l1));
  ASSERT_TRUE(l1.getBoundedIntersection(l0));

  ASSERT_FALSE(l0.getBoundedIntersection(l2));
  ASSERT_FALSE(l2.getBoundedIntersection(l0));

  ASSERT_FALSE(l0.getBoundedIntersection(l3));
  ASSERT_FALSE(l3.getBoundedIntersection(l0));

  ASSERT_FALSE(l1.getBoundedIntersection(l2));
  ASSERT_FALSE(l2.getBoundedIntersection(l1));
}

TEST(Geometry, PolygonSelfIntersection) {
  std::vector<Vec2> points0 = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  Polygon p0(points0);
  ASSERT_FALSE(p0.isSelfIntersecting());

  std::vector<Vec2> points1 = {{0, 0}, {1, 1}, {1, 0}, {0, 1}};
  Polygon p1(points1);
  ASSERT_TRUE(p1.isSelfIntersecting());

  std::vector<Vec2> points2 = {{0, 0}, {4, 0}, {3, 1}, {1, 1}};
  Polygon p2(points2);
  ASSERT_FALSE(p2.isSelfIntersecting());

  std::vector<Vec2> points3 = {{0, 0}, {3, 1}, {4, 0}, {1, 1}};
  Polygon p3(points3);
  ASSERT_TRUE(p3.isSelfIntersecting());
}

TEST(Geometry, PolygonArea) {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({0, 0, 1});
  FacePtr f0 = Face::create(RingVector<VertexPtr>({v0, v2, v1}), {0, 0, -1});
  FacePtr f1 = Face::create(RingVector<VertexPtr>({v0, v1, v3}), {0, -1, 0});
  FacePtr f2 = Face::create(RingVector<VertexPtr>({v0, v3, v2}), {-1, 0, 0});
  FacePtr f3 =
      Face::create(RingVector<VertexPtr>({v1, v2, v3}),
                   {1 / std::sqrt(3), 1 / std::sqrt(3), 1 / std::sqrt(3)});
  ASSERT_FLOAT_EQ(Polygon(f0).getArea(), f0->getArea());
  ASSERT_FLOAT_EQ(Polygon(f1).getArea(), f1->getArea());
  ASSERT_FLOAT_EQ(Polygon(f2).getArea(), f2->getArea());
  ASSERT_FLOAT_EQ(Polygon(f3).getArea(), f3->getArea());
}

TEST(Geometry, PolygonHandedness) {
  // Basic right-handed triangle
  std::vector<Vec2> points0 = {{0, 0}, {1, 0}, {0, 1}};
  Polygon p0(points0);
  Polygon::Handedness h0 = p0.getHandedness();
  ASSERT_EQ(h0, Polygon::Handedness::RIGHT);

  // Basic left-handed triangle
  std::vector<Vec2> points1 = {{0, 0}, {0, 1}, {1, 0}};
  Polygon p1(points1);
  Polygon::Handedness h1 = p1.getHandedness();
  ASSERT_EQ(h1, Polygon::Handedness::LEFT);

  // Right-handed quadrilateral with co-linear points
  std::vector<Vec2> points2 = {{0, 0}, {1, 0}, {2, 0}, {1, 1}};
  Polygon p2(points2);
  Polygon::Handedness h2 = p2.getHandedness();
  ASSERT_EQ(h2, Polygon::Handedness::RIGHT);

  // Basic self-intersecting quadrilateral
  // std::vector<Vec2> points3 = {{0, 0}, {1, 1}, {0, 1}, {1, 0}};
  // Polygon p3(points3);
  // Polygon::Handedness h3 = p3.getHandedness();
  // ASSERT_EQ(h3, Polygon::Handedness::NEITHER);

  // Non-convex shape
  std::vector<Vec2> points4 = {{0, 0}, {5, 0}, {5, 5}, {4, 1}};
  Polygon p4(points4);
  Polygon::Handedness h4 = p4.getHandedness();
  ASSERT_EQ(h4, Polygon::Handedness::RIGHT);
}

// std::vector<std::vector<std::pair<Vec2, std::shared_ptr<Line>>>>
// getNonIntersectingPolygons(const std::vector<std::shared_ptr<Line>>
// &mainLines,
//                            const std::vector<Vec2> &intersections);

// static std::vector<std::vector<std::pair<Vec2, std::shared_ptr<Line>>>>
// splitPolygon(const Polygon &polygon) {
//   std::vector<std::shared_ptr<Line>> lines;
//   for (const auto &line : polygon.getLines()) {
//     lines.push_back(std::make_shared<Line>(*line));
//   }
//   return getNonIntersectingPolygons(lines, polygon.getPoints());
// }

// TEST(Geometry, GetNonIntersectingPolygons) {
//   // Basic triangle
//   std::vector<Vec2> points0 = {{0, 0}, {1, 0}, {0, 1}};
//   auto p0 = splitPolygon(Polygon(points0));
//   ASSERT_EQ(p0.size(), 1);
//   ASSERT_EQ(p0[0].size(), 3);

//   // Basic quadrilateral
//   std::vector<Vec2> points1 = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
//   auto p1 = splitPolygon(Polygon(points1));
//   ASSERT_EQ(p1.size(), 1);
//   ASSERT_EQ(p1[0].size(), 4);

//   // Basic self-intersecting quadrilateral
//   std::vector<Vec2> points2 = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};
//   auto p2 = splitPolygon(Polygon(points2));
//   ASSERT_EQ(p2.size(), 2);
//   ASSERT_EQ(p2[0].size(), 3);
//   ASSERT_EQ(p2[1].size(), 3);

//   // Polygon with two self-intersections on same line
//   std::vector<Vec2> points3 = {{0, 0}, {4, 0}, {4, 2}, {2, -1}, {0, 2}};
//   auto p3 = splitPolygon(Polygon(points3));
//   ASSERT_EQ(p3.size(), 3);
//   ASSERT_EQ(p3[0].size(), 3);
//   ASSERT_EQ(p3[1].size(), 3);
//   ASSERT_EQ(p3[2].size(), 3);

//   // Polygon with self-intersection on a line
//   std::vector<Vec2> points4 = {{0, 0}, {4, 0}, {4, 2}, {2, 0}, {0, 2}};
//   auto p4 = splitPolygon(Polygon(points4));
//   ASSERT_EQ(p4.size(), 2);
//   ASSERT_EQ(p4[0].size(), 3);
//   ASSERT_EQ(p4[1].size(), 3);

//   // Polygon with self-intersection at a vertex
//   std::vector<Vec2> points5 = {{0, 0}, {1, 3}, {4, 4}, {4, 0}, {1, 3}, {0,
//   4}}; auto p5 = splitPolygon(Polygon(points5)); ASSERT_EQ(p5.size(), 2);
//   ASSERT_EQ(p5[0].size(), 3);
//   ASSERT_EQ(p5[1].size(), 3);

//   // Polygon with colinear points
//   std::vector<Vec2> points6 = {{0, 0}, {1, 0}, {2, 0}, {1, 1}};
//   auto p6 = splitPolygon(Polygon(points6));
//   ASSERT_EQ(p6.size(), 1);
//   ASSERT_EQ(p6[0].size(), 4);
// }
