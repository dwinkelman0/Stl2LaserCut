// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <LaserCut.h>
#include <Stl.h>

#include <cmath>
#include <iostream>
#include <numbers>

int main() {
  LaserCutRenderer renderer({4, 0, 3, 2, {400, 400}});
  auto inputFile =
      std::ifstream("/Users/dwinkelman/Downloads/thinker_ascii.stl");
  std::vector<FacePtr> faces = loadFacesFromStl(inputFile);
  std::vector<EdgePtr> edges = collectEdges(faces);
  for (const FacePtr &face : faces) {
    renderer.renderFace(face);
  }
  return 0;
}
