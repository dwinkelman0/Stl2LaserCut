// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <Geometry.h>

#include <fstream>

std::vector<FacePtr> loadFacesFromStl(std::ifstream &inputFile);
