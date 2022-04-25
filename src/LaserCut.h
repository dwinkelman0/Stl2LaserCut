// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <Geometry.h>

class RenderedEdge;
class LaserCutRenderer;

using RenderedEdgePtr = std::shared_ptr<RenderedEdge>;

class RenderedEdge : std::enable_shared_from_this<RenderedEdge> {
 public:
  static RenderedEdgePtr create(const float angle,
                                const BoundedLine &geometricEdge,
                                const BoundedLine &baselineEdge) {
    return std::shared_ptr<RenderedEdge>(
        new RenderedEdge(angle, geometricEdge, baselineEdge));
  }

  BoundedLine normalize() const;
  std::vector<BoundedLine> generateNotches(
      const LaserCutRenderer &renderer,
      std::vector<RenderedEdgePtr> &oppositeEdges) const;

 protected:
  RenderedEdge(const float angle, const BoundedLine &geometricEdge,
               const BoundedLine &baselineEdge)
      : angle_(angle),
        geometricEdge_(geometricEdge),
        baselineEdge_(baselineEdge) {}

 private:
  float angle_;
  BoundedLine geometricEdge_;
  BoundedLine baselineEdge_;
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

  float getGeometricEdgeDistance(const float angle) const;
  float getToothLength(const float angle) const;
  float getMaterialBaseLength(const float angle) const;

 private:
  Config config_;
  std::vector<Polygon> rawPolygons_;
};
