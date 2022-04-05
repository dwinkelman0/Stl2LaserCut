// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Geometry.h>
#include <gtest/gtest.h>

TEST(Geometry, FaceIsPlanar) {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({3, 4, 0});
  VertexPtr v4 = Vertex::create({0, 1, 1});
  FacePtr f0 = Face::create({v0, v1, v2}, {0, 0, 1});
  ASSERT_TRUE(f0->isPlanar());
  FacePtr f1 = Face::create({v0, v1, v3, v2}, {0, 0, 1});
  ASSERT_TRUE(f1->isPlanar());

  // Faces must be planar
  ASSERT_DEATH(
      {
        Face::create({v0, v1, v2}, {0, 1 / std::sqrt(2), 1 / std::sqrt(2)});
      },
      "Assertion failed*");
  ASSERT_DEATH(
      {
        Face::create({v0, v1, v2, v4}, {0, 0, 1});
      },
      "Assertion failed*");
}

TEST(Geometry, FaceArea) {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({2, 1, 0});
  FacePtr f0 = Face::create({v0, v1, v2}, {0, 0, 1});
  FacePtr f1 = Face::create({v0, v3, v2}, {0, 0, 1});
  FacePtr f2 = Face::create({v0, v1, v3, v2}, {0, 0, 1});
  ASSERT_FLOAT_EQ(f0->getArea(), 0.5);
  ASSERT_FLOAT_EQ(f1->getArea(), 1.0);
  ASSERT_FLOAT_EQ(f2->getArea(), 1.5);

  // Cannot create faces with negative area, indicates problem with handedness
  // of order of points
  EXPECT_DEATH(
      {
        Face::create({v0, v2, v3}, {0, 0, 1});
      },
      "Assertion failed *");
}

TEST(Geometry, LineIntersection) {
  // Easy line
  Line l0(1, 1, 1, true);
  Line l1(1, -1, 0, true);
  auto i0 = l0.getIntersection(l1);
  ASSERT_TRUE(i0);
  ASSERT_FLOAT_EQ(std::get<0>(*i0), 0.5);
  ASSERT_FLOAT_EQ(std::get<1>(*i0), 0.5);

  // Trickier line
  Line l2(5, 3, 6, true);
  Line l3(8, -5, 9.6, true);
  auto i1 = l2.getIntersection(l3);
  ASSERT_TRUE(i1);
  ASSERT_FLOAT_EQ(std::get<0>(*i1), 1.2);
  ASSERT_FLOAT_EQ(std::get<1>(*i1), 0);

  // Infinitely many solutions
  Line l4(4, 2, 8, true);
  Line l5(12, 6, 24, true);
  auto i2 = l4.getIntersection(l5);
  ASSERT_FALSE(i2);

  // No solutions
  Line l6(4, 2, 10, true);
  Line l7(12, 6, 24, true);
  auto i3 = l6.getIntersection(l7);
  ASSERT_FALSE(i3);
}

TEST(Geometry, LineOffset) {
  // Easy line
  Line l0(1, 1, 0, true);
  Line c0(1, 1, std::sqrt(2), true);
  ASSERT_TRUE(l0.getOffsetLine(1).getPossibleEquality(c0));
  Line l1(1, 1, 0, false);
  Line c1(1, 1, -std::sqrt(2), false);
  ASSERT_TRUE(l1.getOffsetLine(1).getPossibleEquality(c1));

  // Vertical Line
  Line l2(1, 0, 0, true);
  Line c2(1, 0, 1, true);
  ASSERT_TRUE(l2.getOffsetLine(1).getPossibleEquality(c2));
  Line l3(1, 0, 0, false);
  Line c3(1, 0, -1, false);
  ASSERT_TRUE(l3.getOffsetLine(1).getPossibleEquality(c3));

  // Horizontal Line
  Line l4(0, 1, 0, true);
  Line c4(0, 1, 1, true);
  ASSERT_TRUE(l4.getOffsetLine(1).getPossibleEquality(c4));
  Line l5(0, 1, 0, false);
  Line c5(0, 1, -1, false);
  ASSERT_TRUE(l5.getOffsetLine(1).getPossibleEquality(c5));
}

TEST(Geometry, BoundedLine) {
  BoundedLine l0({2, 0}, {0, 2});
  Line c0(1, 1, 2, true);
  ASSERT_TRUE(c0.getPossibleEquality(l0));

  BoundedLine l1({0, 2}, {2, 0});
  Line c1(1, 1, 2, true);
  ASSERT_TRUE(c1.getPossibleEquality(l1));

  BoundedLine l2({-2, -1}, {2, 1});
  Line c2(1, -2, 0, true);
  ASSERT_TRUE(c2.getPossibleEquality(l2));

  BoundedLine l3({5, 0}, {5, 6});
  Line c3(1, 0, 5, true);
  ASSERT_TRUE(c3.getPossibleEquality(l3));

  BoundedLine l4({0, 5}, {6, 5});
  Line c4(0, 1, 5, true);
  ASSERT_TRUE(c4.getPossibleEquality(l4));

  BoundedLine l5({0, 0}, {1, 0});
  Line c5(0, 1, 0, true);
  ASSERT_TRUE(c5.getPossibleEquality(l5));

  BoundedLine l6({0, 1}, {0, 0});
  Line c6(1, 0, 0, true);
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
  FacePtr f0 = Face::create({v0, v2, v1}, {0, 0, -1});
  FacePtr f1 = Face::create({v0, v1, v3}, {0, -1, 0});
  FacePtr f2 = Face::create({v0, v3, v2}, {-1, 0, 0});
  FacePtr f3 = Face::create(
      {v1, v2, v3}, {1 / std::sqrt(3), 1 / std::sqrt(3), 1 / std::sqrt(3)});
  ASSERT_FLOAT_EQ(Polygon(f0).getArea(), f0->getArea());
  ASSERT_FLOAT_EQ(Polygon(f1).getArea(), f1->getArea());
  ASSERT_FLOAT_EQ(Polygon(f2).getArea(), f2->getArea());
  ASSERT_FLOAT_EQ(Polygon(f3).getArea(), f3->getArea());
}
