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
  FacePtr f1 =
      Face::create({v0, v1, v2}, {0, 1 / std::sqrt(2), 1 / std::sqrt(2)});
  ASSERT_FALSE(f1->isPlanar());
  FacePtr f2 = Face::create({v0, v1, v2, v3}, {0, 0, 1});
  ASSERT_TRUE(f2->isPlanar());
  FacePtr f3 = Face::create({v0, v1, v2, v4}, {0, 0, 1});
  ASSERT_FALSE(f3->isPlanar());
}
