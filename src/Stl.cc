// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Stl.h>

struct VertexPtrComparator {
  bool operator()(const VertexPtr &a, const VertexPtr &b) const {
    if (abs(a->getVector() - b->getVector()) < 1e-6) {
      return false;
    } else {
      return a->getVector() < b->getVector();
    }
  }
};

struct NormalComparator {
  bool operator()(const Vec3 &a, const Vec3 &b) const {
    if (abs(dot(a, b) - 1) < 1e-6) {
      return false;
    } else {
      return a < b;
    }
  }
};

static Vec3 readVectorFromStream(std::istream &inputStream) {
  Vec3 normal;
  inputStream >> std::get<0>(normal) >> std::get<1>(normal) >>
      std::get<2>(normal);
  return normal;
}

/**
 * Generate faces from a set of triangles with common vertices.
 */
std::vector<FacePtr> facesFromTrianglePartition(
    const Vec3 &normal,
    const std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> &triangles) {
  // TODO: algorithm is currently O(n^2), can make O(n log n)
  // TODO: can use std::list for chain instead of std::vector
  std::vector<VertexPtr> chain;
  std::set<VertexPtr> vertexSet;

  // Attempt to find three adjacent vertices found in the triangle
  std::function tryThreeVertexInsert =
      [&chain](const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle) {
        auto &[v0, v1, v2] = triangle;
        for (uint32_t i = 0; i < chain.size() - 2; ++i) {
          if (chain[i] == v2 && chain[i + 1] == v1 && chain[i + 2] == v0) {
            chain.erase(chain.begin() + i + 1);
            return true;
          }
        }
        return false;
      };

  // Attempt to find two adjacent vertices found in the triangle
  std::function tryTwoVertexInsert =
      [&chain, &vertexSet](
          const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle) {
        auto &[v0, v1, v2] = triangle;
        for (uint32_t i = 0; i < chain.size() - 1; ++i) {
          if (chain[i] == v0 && chain[i + 1] == v2) {
            vertexSet.insert(v1);
            chain.insert(chain.begin() + i + 1, v1);
            return true;
          }
        }
        return false;
      };

  // Attempt to find one vertex found in the triangle
  std::function tryOneVertexInsert =
      [&chain, &vertexSet](
          const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle) {
        auto &[v0, v1, v2] = triangle;
        for (uint32_t i = 0; i < chain.size(); ++i) {
          if (chain[i] == v0) {
            vertexSet.insert(v1);
            vertexSet.insert(v2);
            chain.insert(chain.begin() + i, v2);
            chain.insert(chain.begin() + i, v1);
            chain.insert(chain.begin() + i, v0);
            return true;
          }
        }
        return false;
      };

  // If the triangle cannot be inserted into an exiting chain, make a new one
  std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> remainingTriangles(
      triangles);
  chain.push_back(std::get<0>(triangles.back()));
  chain.push_back(std::get<1>(triangles.back()));
  chain.push_back(std::get<2>(triangles.back()));
  chain.push_back(std::get<0>(triangles.back()));
  vertexSet.insert(std::get<0>(triangles.back()));
  vertexSet.insert(std::get<1>(triangles.back()));
  vertexSet.insert(std::get<2>(triangles.back()));
  remainingTriangles.pop_back();
  while (!remainingTriangles.empty()) {
    std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>>
        newRemainingTriangles;
    for (const auto &[v0, v1, v2] : remainingTriangles) {
      if (vertexSet.find(v0) != vertexSet.end() ||
          vertexSet.find(v1) != vertexSet.end() ||
          vertexSet.find(v2) != vertexSet.end()) {
        if (!tryThreeVertexInsert({v0, v1, v2}) &&
            !tryThreeVertexInsert({v1, v2, v0}) &&
            !tryThreeVertexInsert({v2, v0, v1}) &&
            !tryTwoVertexInsert({v0, v1, v2}) &&
            !tryTwoVertexInsert({v1, v2, v0}) &&
            !tryTwoVertexInsert({v2, v0, v1}) &&
            !tryOneVertexInsert({v0, v1, v2}) &&
            !tryOneVertexInsert({v1, v2, v0}) &&
            !tryOneVertexInsert({v2, v0, v1})) {
          std::cout << "There has been a major problem" << std::endl;
        }
      } else {
        newRemainingTriangles.push_back({v0, v1, v2});
      }
    }
    remainingTriangles = newRemainingTriangles;
  }

  std::vector<FacePtr> output;
  RingVector<VertexPtr> remainder(
      std::vector<VertexPtr>(chain.begin(), chain.end() - 1));
  while (remainder.getSize() > 0) {
    const auto [subchain, newRemainder] = remainder.splitOnShortestCycle(
        [](const VertexPtr &a, const VertexPtr &b) { return a == b; });
    if (subchain.getSize() >= 3) {
      output.push_back(Face::create(subchain, normal));
      if (!output.back()) {
        std::cout << "[WARNING] Failed to make face";
      }
    }
    remainder = newRemainder;
  }
  return output;
}

std::vector<FacePtr> facesFromTriangles(
    const Vec3 &normal,
    const std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> &triangles) {
  // Create partitions, i.e. groups of triangles with common vertices
  std::vector<FacePtr> output;
  std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> remainingTriangles(
      triangles);
  while (!remainingTriangles.empty()) {
    std::set<VertexPtr> vertexSet;
    std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> currentTriangles;
    std::function addTriangleToVertexSetFunc =
        [&vertexSet, &currentTriangles](
            const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle,
            bool force) {
          const auto &[v0, v1, v2] = triangle;
          if (vertexSet.find(v0) != vertexSet.end() ||
              vertexSet.find(v1) != vertexSet.end() ||
              vertexSet.find(v2) != vertexSet.end() || force) {
            vertexSet.insert(v0);
            vertexSet.insert(v1);
            vertexSet.insert(v2);
            currentTriangles.push_back(triangle);
            return true;
          } else {
            return false;
          }
        };
    bool initialInsertSuccess =
        addTriangleToVertexSetFunc(remainingTriangles.back(), true);
    assert(initialInsertSuccess);
    remainingTriangles.pop_back();
    bool anyInserted = true;
    while (anyInserted) {
      std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>>
          newRemainingTriangles;
      anyInserted = false;
      for (const auto &triangle : remainingTriangles) {
        if (!addTriangleToVertexSetFunc(triangle, false)) {
          newRemainingTriangles.push_back(triangle);
          anyInserted = true;
        }
      }
      remainingTriangles = newRemainingTriangles;
    }
    std::vector<FacePtr> newFaces =
        facesFromTrianglePartition(normal, currentTriangles);
    output.insert(output.end(), newFaces.begin(), newFaces.end());
  }
  return output;
}

std::vector<FacePtr> loadFacesFromStl(std::ifstream &inputFile) {
  // Parse ASCII STL file to extract triangles
  // Triangles are grouped by having (approximately) the same normal
  NormalComparator normalComparator;
  std::map<Vec3, std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>>,
           NormalComparator>
      normalMap(normalComparator);
  std::set<VertexPtr, VertexPtrComparator> vertexSet;
  std::function newVertexFunc = [&vertexSet](const Vec3 &vec) {
    VertexPtr newVertex = Vertex::create(vec);
    auto [it, success] = vertexSet.insert(newVertex);
    return *it;
  };
  std::string inputText;
  std::getline(inputFile, inputText);
  uint32_t numTriangles = 0;
  while (true) {
    inputFile >> inputText;
    if (inputText == "endsolid") {
      break;
    }
    assert(inputText == "facet");
    inputFile >> inputText;
    assert(inputText == "normal");
    Vec3 normal = readVectorFromStream(inputFile);
    normal = normal / abs(normal);
    inputFile >> inputText >> inputText >> inputText;
    assert(inputText == "vertex");
    Vec3 v0 = readVectorFromStream(inputFile);
    inputFile >> inputText;
    assert(inputText == "vertex");
    Vec3 v1 = readVectorFromStream(inputFile);
    inputFile >> inputText;
    assert(inputText == "vertex");
    Vec3 v2 = readVectorFromStream(inputFile);
    inputFile >> inputText >> inputText;
    assert(inputText == "endfacet");
    auto [it, success] = normalMap.emplace(
        normal, std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>>());
    it->second.emplace_back(newVertexFunc(v0), newVertexFunc(v1),
                            newVertexFunc(v2));
    ++numTriangles;
  }

  // Coalesce triangles into faces
  std::vector<FacePtr> output;
  for (const auto &[normal, triangles] : normalMap) {
    std::vector<FacePtr> faces = facesFromTriangles(normal, triangles);
    output.insert(output.end(), faces.begin(), faces.end());
  }
  for (const FacePtr &face : output) {
    if (!face) {
      std::cout << "Found null face" << std::endl;
    } else {
      face->link();
    }
  }
  std::vector<EdgePtr> edges = collectEdges(output);
  std::cout << "STL summary: " << numTriangles << " triangles, "
            << normalMap.size() << " normals, " << vertexSet.size()
            << " vertices, " << output.size() << " faces, " << edges.size()
            << " edges, "
            << static_cast<int32_t>(output.size() + vertexSet.size()) -
                   static_cast<int32_t>(edges.size())
            << " characteristic number" << std::endl;
  return output;
}
