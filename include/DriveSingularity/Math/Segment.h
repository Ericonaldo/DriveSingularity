#ifndef DRIVE_SINGULARITY_MATH_SEGMENT_H
#define DRIVE_SINGULARITY_MATH_SEGMENT_H

#include "DriveSingularity/Math/Constants.h"
#include "DriveSingularity/Math/Vector.h"

namespace ds {

class Segment {
public:
  Segment(const VectorD &start, const VectorD &end) : start(start), end(end) {}

	const VectorD &getStart() const { return start; }
	const VectorD &getEnd() const { return end; }

private:
  VectorD start, end;
};


inline double signedDistance(const Segment &seg, const VectorD &point) {
	// TODO(ming): refactor distance
  auto segVec = normalize(seg.getEnd() - seg.getEnd());
  auto vec = point - seg.getStart();
  int sign = cross(segVec, normalize(vec)) > -eps ? 1 : -1;
  assert(dot(segVec, normalize(vec)) > -eps);  // TODO
  double proj = dot(vec, segVec);
  return sign * std::sqrt(vec.getLength() * vec.getLength() - proj * proj);
}

inline bool isIntersect(const Segment &seg1, const Segment &seg2) {
  auto segVec = seg1.getEnd() - seg1.getStart();
  auto vec1 = seg2.getStart() - seg1.getStart(), vec2 = seg2.getEnd() - seg1.getStart();
  double cross1 = cross(vec1, segVec), cross2 = cross(vec2, segVec);
  if (fabs(cross1) < eps) {
    return fabs(segVec.getLength() - (segVec - vec1).getLength() - vec1.getLength()) < eps;
  }
  if (fabs(cross2) < eps) {
    return fabs(segVec.getLength() - (segVec - vec2).getLength() - vec2.getLength()) < eps;
  }
  if (cross1 * cross2 > 0) return false;

  segVec = seg2.getEnd() - seg2.getStart();
  vec1 = seg1.getStart() - seg2.getStart();
  vec2 = seg1.getEnd() - seg2.getStart();
  cross1 = cross(vec1, segVec);
  cross2 = cross(vec2, segVec);
  if (fabs(cross1) < eps) {
    return fabs(segVec.getLength() - (segVec - vec1).getLength() - vec1.getLength()) < eps;
  }
  if (fabs(cross2) < eps) {
    return fabs(segVec.getLength() - (segVec - vec2).getLength() - vec2.getLength()) < eps;
  }
  return cross1 * cross2 < 0;
}

} // namespace ds

#endif // DRIVE_SINGULARITY_MATH_SEGMENT_H
