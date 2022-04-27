// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <vector>

template <typename T>
std::pair<RingVector<T>, RingVector<T>> getSmallestClosedShape(
    const RingVector<T> &chain,
    const std::function<bool(const T &, const T &)> &equalityFunc) {
  assert(chain.size() >= 3);
  uint32_t chainSize = chain.size();
  uint32_t begin = 0, end = chainSize;
  for (uint32_t i = 0; i < chainSize; ++i) {
    for (uint32_t j = i + 1; j < i + 1 + chainSize; ++j) {
      if (equalityFunc(chain[i], chain[j % chainSize])) {
        if (j - i < end - begin) {
          begin = i;
          end = j;
        }
        break;
      }
    }
  }
  end %= chainSize;
  auto beginIt = chain.begin() + begin;
  auto endIt = chain.begin() + end % chainSize;
  if (begin < end) {
    std::vector<T> subchain(beginIt, endIt);
    std::vector<T> remainder(chain.begin(), beginIt);
    remainder.insert(remainder.end(), endIt, chain.end());
    return std::pair<std::vector<T>, std::vector<T>>(subchain, remainder);
  } else {
    std::vector<T> subchain(chain.begin(), endIt);
    subchain.insert(subchain.end(), beginIt, chain.end());
    std::vector<T> remainder(endIt, beginIt);
    return std::pair<std::vector<T>, std::vector<T>>(subchain, remainder);
  }
};
