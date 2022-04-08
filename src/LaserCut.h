// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <Geometry.h>

class LaserCutRenderer {
 public:
  struct Config {
    float minimumToothWidth;
    float toothClearance;
    float materialThickness;
    float minimumLayoutSpacing;
    Vec2 materialDimensions;
  };

  enum class FaceRenderStatus { SUCCESS, UNATTACHED, SELF_INTERSECTING };

  LaserCutRenderer(const Config &config);

  float getGeometricEdgeDistance(const float angle) const;
  float getToothLength(const float angle) const;
  float getMaterialBaseLength(const float angle) const;

 private:
  Config config_;
  std::vector<Polygon> rawPolygons_;
};
