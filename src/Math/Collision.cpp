#include "Math/Collision.h"

#include <array>
#include <cmath>
#include <vector>

#include "Math/Constants.h"
#include "Math/Interval.h"

namespace ds {
// Projections
namespace {

Interval project(const VectorD &axis, const Segment &seg) {
  assert(std::fabs(axis.getLength() - 1) < eps);
  double l = dot(axis, seg.getStart());
  double r = dot(axis, seg.getEnd());
  if (l > r)
    std::swap(l, r);
  return {l, r};
}

Interval project(const VectorD &axis, const Obb &obb) {
  assert(std::fabs(axis.getLength() - 1) < eps);

  double l = 0x7fffffff, r = -0x7fffffff;
  auto vertices = obb.getVertices();
  for (auto &vertex : vertices) {
    auto len = dot(vertex, axis);
    if (len < l)
      l = len;
    if (len > r)
      r = len;
  }
  return {l, r};
}

} // namespace

// We don't care the co-linear cases
bool checkCollision(const Segment &s1, const Segment &s2) {
  auto v1 = s1.getEnd() - s1.getStart();
  double area1 = cross(v1, s2.getStart() - s1.getStart());
  double area2 = cross(v1, s2.getEnd() - s1.getStart());
  if (area1 * area2 > 0)
    return false;

  v1 = s2.getEnd() - s2.getStart();
  area1 = cross(v1, s1.getStart() - s2.getStart());
  area2 = cross(v1, s1.getEnd() - s2.getStart());
  return area1 * area2 <= 0;
}

/**
 * Check collision between two abstract objects.
 *
 * Provide observations of objects when you wanna check collision between them.
 *
 * @param obb1 observation of object 1
 * @param obb2 observation of object 2
 * @return collision or not
 */
bool checkCollision(const Obb &obb1, const Obb &obb2) {
  // TODO: optimization
  auto vertices1 = obb1.getVertices();
  auto vertices2 = obb2.getVertices();
  std::array<VectorD, 4> axes = {normalize(vertices1[0] - vertices1[1]),
                                 normalize(vertices1[2] - vertices1[1]),
                                 normalize(vertices2[0] - vertices2[1]),
                                 normalize(vertices2[2] - vertices2[1])};

  for (auto &axis : axes) {
    if (!checkIntervalOverlapping(project(axis, obb1), project(axis, obb2)))
      return false;
  }
  return true;
}

bool checkCollision(const Obb &obb, const Segment &seg) {
  auto vertices = obb.getVertices();
  std::array<VectorD, 4> axes = {
      normalize(vertices[0] - vertices[1]),
      normalize(vertices[3] - vertices[0]),
      normalize(seg.getEnd() - seg.getStart()),
      normalize(rotate90(seg.getEnd() - seg.getStart()))};

  for (auto &axis : axes) {
    if (!checkIntervalOverlapping(project(axis, obb), project(axis, seg)))
      return false;
  }
  return true;
}

bool checkCollision(const Segment &seg, const Obb &obb) {
  return checkCollision(obb, seg);
}

} // namespace ds