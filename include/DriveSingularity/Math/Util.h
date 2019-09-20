#ifndef DRIVE_SINGULARITY_UTIL_H
#define DRIVE_SINGULARITY_UTIL_H

#include <utility>

namespace ds {

inline double remap(double value, const std::pair<double, double> &originalRange,
             const std::pair<double, double> &targetRange) {
  return originalRange.first +
         (targetRange.second - targetRange.first) /
             (originalRange.second - originalRange.first) *
             (value - originalRange.first);
}

} // namespace ds

#endif // DRIVE_SINGULARITY_UTIL_H
