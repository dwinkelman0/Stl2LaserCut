// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Stl.h>
#include <gtest/gtest.h>

std::vector<FacePtr> facesFromTrianglePartition(
    const Vec3 &normal,
    const std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> &triangles);

TEST(Stl, FacesFromTrianglePartition) {
  // Test simple triangle
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  std::vector<FacePtr> f0 =
      facesFromTrianglePartition({0, 0, 1}, {{v0, v1, v2}});
  ASSERT_EQ(f0.size(), 1);

  // Test simple quadrilateral (2-vertex insert)
  VertexPtr v3 = Vertex::create({1, 1, 0});
  std::vector<FacePtr> f1 =
      facesFromTrianglePartition({0, 0, 1}, {{v0, v1, v2}, {v1, v3, v2}});
  ASSERT_EQ(f1.size(), 1);

  // Test two triangles with same normal (new chain)
  VertexPtr v4 = Vertex::create({2, 1, 0});
  VertexPtr v5 = Vertex::create({1, 2, 0});

  // Test two triangles with single overlapping point (1-vertex insert)
  std::vector<FacePtr> f3 =
      facesFromTrianglePartition({0, 0, 1}, {{v0, v1, v2}, {v2, v4, v5}});
  ASSERT_EQ(f3.size(), 2);

  // Test three triangles with three overlapping points (3-vertex insert)
  std::vector<FacePtr> f4 = facesFromTrianglePartition(
      {0, 0, 1}, {{v1, v4, v3}, {v3, v4, v5}, {v1, v3, v2}, {v2, v3, v5}});
  ASSERT_EQ(f4.size(), 1);
  ASSERT_EQ(f4.front()->getVertices().size(), 4);
}
