#ifndef DRIVE_SINGULARITY_MATH_CONSTANTS_H
#define DRIVE_SINGULARITY_MATH_CONSTANTS_H

#include <cmath>

namespace ds {

constexpr double Pi = 3.14159265358979323846;

constexpr double Inf = 0x7fffffff;

constexpr double eps = 1e-3;

inline bool doubleEq(double x, double y) { return std::fabs(x - y) < eps; }

inline double notZero(double x) {
  if (std::fabs(x) >= eps) return x;
  return x > 0 ? eps : -eps;
}

inline double clip(double x, double l, double r) {
  if (x < l) return l;
  if (x > r) return r;
  return x;
}

inline double wrapToPi(double x) {
  while (x > Pi) x -= 2 * Pi;
  while (x < -Pi) x += 2 * Pi;
  return x;
}

} // namespace ds

#endif // DRIVE_SINGULARITY_ENGINE_MATH_CONSTANTS_H
