// Copyright 2022 by Daniel Winkelman. All rights reserved.

#pragma once

#include <iostream>
#include <vector>

template <typename T>
class RingVector {
 public:
  RingVector(const std::vector<T> &vec) : vec_(vec) {}

  uint32_t getSize() const { return vec_.size(); }

  T &operator[](const uint32_t index) { return vec_[index % getSize()]; }
  const T &operator[](const uint32_t index) const {
    return vec_[index % getSize()];
  }

  void foreach (const std::function<void(const T &)> &callback) const {
    for (uint32_t i = 0; i < getSize(); ++i) {
      callback(this->operator[](i));
    }
  }

  template <typename T2>
  RingVector<T2> foreach (const std::function<T2(const T &)> &callback) const {
    std::vector<T2> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      output.push_back(callback(this->operator[](i)));
    }
    return output;
  }

  void foreachPair(
      const std::function<void(const T &, const T &)> &callback) const {
    for (uint32_t i = 0; i < getSize(); ++i) {
      callback(this->operator[](i), this->operator[](i + 1));
    }
  }

  template <typename T2>
  RingVector<T2> foreachPair(
      const std::function<T2(const T &, const T &)> &callback) const {
    std::vector<T2> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      output.push_back(callback(this->operator[](i), this->operator[](i + 1)));
    }
    return RingVector<T2>(output);
  }

  template <typename T2, typename T3>
  RingVector<T3> zip(
      const RingVector<T2> &other,
      const std::function<T3(const T &, const T2 &)> &callback) const {
    assert(getSize() == other.getSize());
    std::vector<T3> output;
    for (uint32_t i = 0; i < getSize(); ++i) {
      output.push_back(callback(this->operator[](i), other[i]));
    }
    return RingVector<T3>(output);
  }

  std::pair<RingVector<T>, RingVector<T>> splitOnShortestCycle(
      const std::function<bool(const T &, const T &)> &equalityFunc) const {
    uint32_t begin = 0;
    uint32_t end = getSize();
    for (uint32_t i = 0; i < getSize(); ++i) {
      for (uint32_t j = i + 1; j < i + getSize(); ++j) {
        if (equalityFunc(this->operator[](i), this->operator[](j)) &&
            j - i < end - begin) {
          begin = i;
          end = j;
          assert(begin < end);
        }
      }
    }
    return {slice(begin, end), slice(end, begin + getSize())};
  }

  RingVector<T> slice(const uint32_t begin, const uint32_t end) const {
    uint32_t normBegin = begin % getSize();
    uint32_t normEnd = end % getSize();
    if (normBegin == normEnd) {
      return end - begin == 0 ? RingVector({}) : *this;
    } else if (normBegin < normEnd) {
      return RingVector(
          std::vector<T>(vec_.begin() + normBegin, vec_.begin() + normEnd));
    } else {
      std::vector<T> output(vec_.begin() + normBegin, vec_.end());
      output.insert(output.end(), vec_.begin(), vec_.begin() + normEnd);
      return RingVector(output);
    }
  }

 private:
  std::vector<T> vec_;
};
