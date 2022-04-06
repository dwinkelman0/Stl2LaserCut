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

  // Test simple quadrilateral
  VertexPtr v3 = Vertex::create({1, 1, 0});
  std::vector<FacePtr> f1 =
      facesFromTrianglePartition({0, 0, 1}, {{v0, v1, v2}, {v1, v3, v2}});
  ASSERT_EQ(f1.size(), 1);

  // Test two triangles with same normal
  VertexPtr v4 = Vertex::create({2, 1, 0});
  VertexPtr v5 = Vertex::create({1, 2, 0});

  // Test two triangles with single overlapping point
  std::vector<FacePtr> f3 =
      facesFromTrianglePartition({0, 0, 1}, {{v0, v1, v2}, {v2, v4, v5}});
  ASSERT_EQ(f3.size(), 2);
}
