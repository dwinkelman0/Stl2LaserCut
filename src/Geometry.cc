// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include "Geometry.h"

#include <iostream>

Vec3 operator-(const Vec3 vec) {
  return {-std::get<0>(vec), -std::get<1>(vec), -std::get<2>(vec)};
}

Vec3 operator-(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<0>(vec1) - std::get<0>(vec2),
          std::get<1>(vec1) - std::get<1>(vec2),
          std::get<2>(vec1) - std::get<2>(vec2)};
}

Vec3 operator/(const Vec3 vec, const float x) {
  return {std::get<0>(vec) / x, std::get<1>(vec) / x, std::get<2>(vec) / x};
}

float dot(const Vec3 vec1, const Vec3 vec2) {
  return std::get<0>(vec1) * std::get<0>(vec2) +
         std::get<1>(vec1) * std::get<1>(vec2) +
         std::get<2>(vec1) * std::get<2>(vec2);
}

Vec3 cross(const Vec3 vec1, const Vec3 vec2) {
  return {std::get<1>(vec1) * std::get<2>(vec2) -
              std::get<2>(vec1) * std::get<1>(vec2),
          std::get<2>(vec1) * std::get<0>(vec2) -
              std::get<0>(vec1) * std::get<2>(vec2),
          std::get<0>(vec1) * std::get<1>(vec2) -
              std::get<1>(vec1) * std::get<0>(vec2)};
}

float abs(const Vec3 vec) {
  return std::sqrt(std::get<0>(vec) * std::get<0>(vec) +
                   std::get<1>(vec) * std::get<1>(vec) +
                   std::get<2>(vec) * std::get<2>(vec));
}

float angle(const Vec3 vec1, const Vec3 vec2) {
  return std::acos(dot(vec1, vec2) / abs(vec1) / abs(vec2));
}

void Vertex::link(const FacePtr &face, const VertexPtr &other) {
  auto newEdge = Edge::create(this->shared_from_this(), other);
  auto [it1, success1] = edges_.emplace(other, newEdge);
  auto [it2, success2] =
      other->edges_.emplace(this->shared_from_this(), newEdge);
  assert(success1 == success2);
  if (success1) {
    it1->second->leftFace_ = face;
  } else {
    it1->second->rightFace_ = face;
  }
}

float Edge::getAngle() const {
  assert(leftFace_);
  assert(rightFace_);
  return angle(leftFace_->getNormal(), rightFace_->getNormal());
}

bool Edge::isConvex() const {
  assert(leftFace_);
  assert(rightFace_);
  Vec3 vector = v2_->getVector() - v1_->getVector();
  Vec3 product = cross(leftFace_->getNormal(), rightFace_->getNormal());
  float alignment = dot(vector, product);
  return alignment > 0;
}

void Face::link() {
  for (uint32_t i = 0; i < vertices_.size() - 1; ++i) {
    vertices_[i]->link(this->shared_from_this(), vertices_[i + 1]);
  }
  vertices_.back()->link(this->shared_from_this(), vertices_.front());
}

bool Face::isPlanar() const {
  Vec3 basisEdge = vertices_[1]->getVector() - vertices_[0]->getVector();
  for (uint32_t i = 2; i < vertices_.size(); ++i) {
    Vec3 secondEdge = vertices_[i]->getVector() - vertices_[0]->getVector();
    Vec3 product = cross(basisEdge, secondEdge);
    product = product / abs(product);
    float alignment = dot(product, normal_);
    if (abs(alignment - 1) > 1e-6) {
      return false;
    }
  }
  return true;
}

std::set<EdgePtr> collectEdges(const std::vector<FacePtr> &faces) {
  std::set<EdgePtr> edges;
  for (const FacePtr &face : faces) {
    for (const VertexPtr &vertex : face->vertices_) {
      for (const auto &[other, edge] : vertex->edges_) {
        edges.insert(edge);
      }
    }
  }
  return edges;
}
