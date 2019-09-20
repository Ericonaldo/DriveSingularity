#ifndef DRIVE_SINGULARITY_MATH_INTERVAL_H
#define DRIVE_SINGULARITY_MATH_INTERVAL_H

#include <cassert>

namespace ds {

class Interval {
public:
  Interval(double l, double r) : l(l), r(r) { assert(l <= r); }

  const double getL() const { return l; }

  const double getR() const { return r; }

private:
  double l, r;
};

// Return whether two intervals overlap
inline bool checkIntervalOverlapping(const Interval &i1, const Interval &i2) {
  if (i1.getL() <= i2.getL())
    return i1.getR() >= i2.getL();
  return i2.getR() >= i1.getL();
}

} // namespace ds

#endif // DRIVE_SINGULARITY_MATH_INTERVAL_H
