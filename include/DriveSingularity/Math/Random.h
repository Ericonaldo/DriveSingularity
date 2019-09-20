#ifndef DRIVE_SINGULARITY_RANDOM_H
#define DRIVE_SINGULARITY_RANDOM_H

#include <random>
#include <type_traits>
#include <utility>

namespace ds {

template <class T = double> T random(T l, T r) {
  static std::mt19937_64 eng(std::random_device{}());
  using DistType = std::conditional_t<std::is_integral<T>::value,
                                      std::uniform_int_distribution<T>,
                                      std::uniform_real_distribution<T>>;
  DistType dist(l, r);
  return dist(eng);
}

template <class T = double> T random(const std::pair<T, T> &range) {
  return random(range.first, range.second);
}

} // namespace ds

#endif // DRIVE_SINGULARITY_RANDOM_H
