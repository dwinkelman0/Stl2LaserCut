// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

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

Vec3 operator-(const Vec3 vec);
Vec2 operator-(const Vec2 vec1, const Vec2 vec2);
Vec3 operator-(const Vec3 vec1, const Vec3 vec2);
Vec3 operator/(const Vec3 vec, const float x);
float dot(const Vec3 vec1, const Vec3 vec2);
float cross(const Vec2 vec1, const Vec2 vec2);
Vec3 cross(const Vec3 vec1, const Vec3 vec2);
float abs(const Vec3 vec);
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

  friend std::set<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);

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

  float getAngle() const;
  bool isConvex() const;

 protected:
  Edge(const VertexPtr &v1, const VertexPtr &v2) : v1_(v1), v2_(v2) {}

 private:
  VertexPtr v1_;
  VertexPtr v2_;
  FacePtr leftFace_;
  FacePtr rightFace_;
};

class Face : public std::enable_shared_from_this<Face> {
 public:
  static FacePtr create(const std::vector<VertexPtr> &vertices,
                        const Vec3 &normal) {
    FacePtr output(new Face(vertices, normal));
    if (output->getArea() <= 0 || !output->isPlanar()) {
      return nullptr;
    }
    return output;
  }

  void link();

  inline Vec3 getNormal() const { return normal_; }
  inline std::vector<VertexPtr> getVertices() const { return vertices_; }
  bool isPlanar() const;
  float getArea() const;

  friend std::set<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);

 protected:
  Face(const std::vector<VertexPtr> &vertices, const Vec3 &normal)
      : vertices_(vertices), normal_(normal / abs(normal)) {
    assert(vertices_.size() >= 3);
  }

 private:
  std::vector<VertexPtr> vertices_;
  Vec3 normal_;
};

class Line {
 public:
  Line(const float a, const float b, const float c, const float direction)
      : a_(a), b_(b), c_(c), direction_(direction) {}

  std::optional<Vec2> getIntersection(const Line &other) const;
  Line getOffsetLine(const float offset) const;
  bool getPossibleEquality(const Line &other) const;

  friend std::ostream &operator<<(std::ostream &os, const Line &line);

 protected:
  float a_, b_, c_;
  bool direction_;
};

class BoundedLine : public Line {
 public:
  BoundedLine(const Vec2 b1, const Vec2 b2);

  std::optional<Vec2> getBoundedIntersection(const BoundedLine &other) const;

  friend std::ostream &operator<<(std::ostream &os, const BoundedLine &line);

 private:
  Vec2 lowerBound_, upperBound_;
};

class Polygon {
 public:
  Polygon(const std::vector<Vec2> &points) : points_(points) {}
  Polygon(const FacePtr &face);

  bool isSelfIntersecting() const;
  float getArea() const;

 private:
  std::vector<Vec2> points_;
};

std::set<EdgePtr> collectEdges(const std::vector<FacePtr> &faces);
