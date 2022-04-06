// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Stl.h>

#include <cmath>
#include <iostream>
#include <numbers>

int main() {
  auto inputFile =
      std::ifstream("/Users/dwinkelman/Downloads/thinker_ascii.stl");
  loadFacesFromStl(inputFile);
  return 0;
}
