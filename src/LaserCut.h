// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <Geometry.h>

class RenderedEdge;
class LaserCutRenderer;

class RenderedEdge : std::enable_shared_from_this<RenderedEdge> {
 public:
  RenderedEdge(const EdgePtr &edge, const BoundedLine &geometricEdge)
      : edge_(edge), geometricEdge_(geometricEdge) {}

 private:
  EdgePtr edge_;
  BoundedLine geometricEdge_;
};

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

  std::vector<std::vector<RenderedEdge>> renderFace(const FacePtr &face) const;

 protected:
  /**
   * Get distance between geometric edge (i.e. derived from STL) and the
   * baseline at which the edge meets another edge.
   */
  float f1(const float angle) const;

  /**
   * Get the distance between the baseline and the depth of the teeth.
   */
  float f2(const float angle) const;

  /**
   * Get the length of the teeth.
   */
  float f3(const float angle) const;

 private:
  Config config_;
  std::vector<Polygon> rawPolygons_;
};
