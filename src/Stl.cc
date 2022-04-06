// Copyright 2022 by Daniel Winkelman. All rights reserved.

#include <Stl.h>

struct VertexPtrComparator {
  bool operator()(const VertexPtr &a, const VertexPtr &b) const {
    return a->getVector() < b->getVector();
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
  std::vector<std::vector<VertexPtr>> chains;

  // Attempt to find two adjacent vertices found in the triangle
  std::function tryTwoVertexInsert =
      [&chains](const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle) {
        auto &[v0, v1, v2] = triangle;
        for (std::vector<VertexPtr> &chain : chains) {
          for (uint32_t i = 0; i < chain.size() - 1; ++i) {
            if (chain[i] == v0 && chain[i + 1] == v2) {
              std::cout << "2-vertex insert" << std::endl;
              chain.insert(chain.begin() + i + 1, v1);
              return true;
            }
          }
        }
        return false;
      };

  // Attempt to find one vertex found in the triangle
  std::function tryOneVertexInsert =
      [&chains](const std::tuple<VertexPtr, VertexPtr, VertexPtr> &triangle) {
        auto &[v0, v1, v2] = triangle;
        for (std::vector<VertexPtr> &chain : chains) {
          for (uint32_t i = 0; i < chain.size(); ++i) {
            if (chain[i] == v0) {
              std::cout << "1-vertex insert" << std::endl;
              chain.insert(chain.begin() + i, v2);
              chain.insert(chain.begin() + i, v1);
              chain.insert(chain.begin() + i, v0);
              return true;
            }
          }
        }
        return false;
      };

  // If the triangle cannot be inserted into an exiting chain, make a new one
  for (const auto &[v0, v1, v2] : triangles) {
    if (!tryTwoVertexInsert({v0, v1, v2}) &&
        !tryTwoVertexInsert({v1, v2, v0}) &&
        !tryTwoVertexInsert({v2, v0, v1}) &&
        !tryOneVertexInsert({v0, v1, v2}) &&
        !tryOneVertexInsert({v1, v2, v0}) &&
        !tryOneVertexInsert({v2, v0, v1})) {
      std::cout << "new chain insert" << std::endl;
      chains.push_back({v0, v1, v2, v0});
    }
  }

  // Try to find the smallest closed shape within a chain
  std::function smallestClosedShapeFunc =
      [](const std::vector<VertexPtr> &chain) {
        assert(chain.size() >= 4);
        assert(chain.front() == chain.back());
        std::vector<VertexPtr>::const_iterator begin = chain.begin(),
                                               end = chain.end() - 1;
        for (uint32_t i = 0; i < chain.size(); ++i) {
          for (uint32_t j = i + 3; j < chain.size(); ++j) {
            if (chain[i] == chain[j]) {
              if (j - i < end - begin) {
                std::cout << i << " -> " << j << std::endl;
                begin = chain.begin() + i;
                end = chain.begin() + j;
              }
              break;
            }
          }
        }
        std::vector<VertexPtr> subchain(begin, end);
        std::vector<VertexPtr> remainder(chain.begin(), begin);
        remainder.insert(remainder.end(), end, chain.end());
        return std::pair<std::vector<VertexPtr>, std::vector<VertexPtr>>(
            subchain, remainder);
      };

  std::vector<FacePtr> output;
  for (const std::vector<VertexPtr> &chain : chains) {
    std::vector<VertexPtr> remainder(chain);
    while (remainder.size() > 1) {
      const auto [subchain, newRemainder] = smallestClosedShapeFunc(remainder);
      std::cout << "subchain: " << subchain.size() << std::endl;
      output.push_back(Face::create(subchain, normal));
      remainder = newRemainder;
    }
  }
  return output;
}

std::vector<FacePtr> facesFromTriangles(
    const Vec3 &normal,
    const std::vector<std::tuple<VertexPtr, VertexPtr, VertexPtr>> &triangles) {
  // Create partitions, i.e. groups of triangles with common vertices
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
  }
  std::cout << normalMap.size() << " normals" << std::endl;
  std::cout << vertexSet.size() << " vertices" << std::endl;

  // Coalesce triangles into faces
  std::vector<FacePtr> output;
  for (const auto &[normal, triangles] : normalMap) {
    std::vector<FacePtr> faces = facesFromTriangles(normal, triangles);
    output.insert(output.end(), faces.begin(), faces.end());
  }
  std::cout << output.size() << " faces" << std::endl;
  return output;
}
