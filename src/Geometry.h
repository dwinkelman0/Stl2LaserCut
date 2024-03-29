// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <RingVector.h>
#include <assert.h>

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

class Vertex;
using VertexPtr = std::shared_ptr<Vertex>;
class Edge;
using EdgePtr = std::shared_ptr<Edge>;
class Face;
using FacePtr = std::shared_ptr<Face>;
using Vec2 = std::tuple<float, float>;
using Vec3 = std::tuple<float, float, float>;
class LaserCutRenderer;

Vec3 operator-(const Vec3 vec);
Vec2 operator+(const Vec2 vec1, const Vec2 vec2);
Vec2 operator-(const Vec2 vec1, const Vec2 vec2);
Vec3 operator-(const Vec3 vec1, const Vec3 vec2);
Vec2 operator/(const Vec2 vec, const float x);
Vec3 operator/(const Vec3 vec, const float x);
float dot(const Vec2 vec1, const Vec2 vec2);
float dot(const Vec3 vec1, const Vec3 vec2);
float cross(const Vec2 vec1, const Vec2 vec2);
Vec3 cross(const Vec3 vec1, const Vec3 vec2);
float abs(const Vec2 vec);
float abs(const Vec3 vec);
float angle(const Vec2 vec1, const Vec2 vec2);
float angle(const Vec3 vec1, const Vec3 vec2);
Vec3 rotateX(const Vec3 vec, const float angle);
Vec3 rotateY(const Vec3 vec, const float angle);
Vec3 rotateZ(const Vec3 vec, const float angle);

std::ostream &operator<<(std::ostream &os, const Vec2 &vec);
std::ostream &operator<<(std::ostream &os, const Vec3 &vec);

class Vertex : public std::enable_shared_from_this<Vertex> {
 public:
  static VertexPtr create(const Vec3 vec) { return VertexPtr(new Vertex(vec)); }

  void link(const FacePtr &face, const VertexPtr &other);

  inline Vec3 getVector() const { return vec_; }
  EdgePtr getEdge(const VertexPtr other) const;

  friend std::vector<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);

 protected:
  Vertex(const Vec3 vec) : vec_(vec) {}

 private:
  Vec3 vec_;
  std::map<VertexPtr, EdgePtr> edges_;
};

class Edge : public std::enable_shared_from_this<Edge> {
  friend class Vertex;

 public:
  static EdgePtr create(const VertexPtr &v1, const VertexPtr &v2) {
    return EdgePtr(new Edge(v1, v2));
  }

  inline std::pair<VertexPtr, VertexPtr> getVertices() const {
    return {v1_, v2_};
  }
  float getAngle() const;
  bool isConvex() const;

 protected:
  Edge(const VertexPtr &v1, const VertexPtr &v2)
      : v1_(v1), v2_(v2), leftFace_(nullptr), rightFace_(nullptr) {}

 private:
  VertexPtr v1_;
  VertexPtr v2_;
  FacePtr leftFace_;
  FacePtr rightFace_;
};

class Face : public std::enable_shared_from_this<Face> {
 public:
  struct BaselineEdge {
    float length;
    float horizontalOffset;
    float verticalOffset;
  };

  static FacePtr create(const RingVector<VertexPtr> &vertices,
                        const Vec3 &normal) {
    FacePtr output(new Face(vertices, normal));
    if (output->getArea() <= 0 || !output->isPlanar()) {
      return nullptr;
    }
    return output;
  }

  void link();

  inline Vec3 getNormal() const { return normal_; }
  inline RingVector<VertexPtr> getVertices() const { return vertices_; }
  RingVector<EdgePtr> getEdges() const;
  bool isPlanar() const;
  float getArea() const;

  friend std::vector<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);

 protected:
  Face(const RingVector<VertexPtr> &vertices, const Vec3 &normal)
      : vertices_(vertices), normal_(normal / abs(normal)) {
    assert(vertices_.getSize() >= 3);
  }

 private:
  RingVector<VertexPtr> vertices_;
  Vec3 normal_;
};

class Line {
 public:
  Line(const float a, const float b, const float c) : a_(a), b_(b), c_(c) {
    float magnitude = std::sqrt(a_ * a_ + b_ * b_);
    a_ /= magnitude;
    b_ /= magnitude;
    c_ /= magnitude;
  }

  Line operator-() const { return Line(-a_, -b_, -c_); }

  std::optional<Vec2> getIntersection(const Line &other) const;
  Line getOffsetLine(const float offset) const;
  bool getPossibleEquality(const Line &other) const;
  Line getPerpendicularLine(const Vec2 &point) const;
  bool comparePoints(const Vec2 &a, const Vec2 &b) const;
  Line normalize() const;
  float getAngle(const Line &other) const;

  friend std::ostream &operator<<(std::ostream &os, const Line &line);

  class LinePointComparator {
   public:
    LinePointComparator(const Line *line) : line_(line) {}
    bool operator()(const Vec2 &a, const Vec2 &b) const {
      return line_->comparePoints(a, b);
    }

   private:
    const Line *line_;
  };

  LinePointComparator getPointComparator() const {
    return LinePointComparator(this);
  }

 protected:
  float a_, b_, c_;
};

class BoundedLine : public Line {
 public:
  BoundedLine(const Vec2 b1, const Vec2 b2);
  BoundedLine(const Line &line, const Vec2 &b1, const Vec2 &b2);

  inline Vec2 getLowerBound() const { return lowerBound_; }
  inline Vec2 getUpperBound() const { return upperBound_; }

  friend std::ostream &operator<<(std::ostream &os, const BoundedLine &line);

 private:
  Vec2 lowerBound_, upperBound_;
};

class Polygon {
 public:
  Polygon(const RingVector<Vec2> &points) : points_(points) {}
  template <typename T>
  Polygon(const RingVector<std::pair<Vec2, T>> &points)
      : points_(points.template foreach<Vec2>(
            [](const std::pair<Vec2, T> &a) { return a.first; })) {}
  Polygon(const FacePtr &face);

  enum class Handedness { RIGHT, LEFT, NEITHER };

  inline RingVector<Vec2> getPoints() const { return points_; }
  float getArea() const;
  Handedness getHandedness() const;
  RingVector<BoundedLine> getLines() const;

  friend std::ostream &operator<<(std::ostream &os, const Polygon &polygon);

 private:
  RingVector<Vec2> points_;
};

std::vector<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);
