// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <cmath>
#include <iostream>
#include <numbers>

#include "Geometry.h"

int main() {
  VertexPtr v0 = Vertex::create({0, 0, 0});
  VertexPtr v1 = Vertex::create({1, 0, 0});
  VertexPtr v2 = Vertex::create({0, 1, 0});
  VertexPtr v3 = Vertex::create({0, 0, 1});
  FacePtr f0 = Face::create({v0, v2, v1}, {0, 0, -1});
  FacePtr f1 = Face::create({v0, v1, v3}, {0, -1, 0});
  FacePtr f2 = Face::create({v0, v3, v2}, {-1, 0, 0});
  FacePtr f3 = Face::create(
      {v1, v2, v3}, {1 / std::sqrt(3), 1 / std::sqrt(3), 1 / std::sqrt(3)});
  std::vector<FacePtr> faces({f0, f1, f2, f3});

  bool allFacesPlanar = true;
  for (const FacePtr &face : faces) {
    face->link();
    if (!face->isPlanar()) {
      std::cout << "Face is not planar" << std::endl;
      allFacesPlanar = false;
    }
  }

  std::set<EdgePtr> edges = collectEdges(faces);
  for (const EdgePtr edge : edges) {
    std::cout << edge->getAngle() * 180 / std::numbers::pi << " degrees "
              << (edge->isConvex() ? "convex" : "concave") << std::endl;
  }
}
