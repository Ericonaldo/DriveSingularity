#include "Math/Obb.h"

#include <array>
#include <cmath>

#include "Math/Interval.h"
#include "Math/Constants.h"

namespace ds {

std::array<VectorD, 4> Obb::getVertices() const {
  // First we assume that \a rotation is zero and compute the
  // vertices. Then we rotate the results.
  std::array<VectorD, 4> res = {
      VectorD(getX() + halfLength, getY() + halfWidth),
      VectorD(getX() - halfLength, getY() + halfWidth),
      VectorD(getX() - halfLength, getY() - halfWidth),
      VectorD(getX() + halfLength, getY() - halfWidth),
  };

  auto cos = std::cos(rotation), sin = std::sin(rotation);
  for (auto &v : res) {
    // the coordinate of the vertex relative to the center
    auto vx = v.getX() - getX(), vy = v.getY() - getY();

    v = VectorD(getX() + (vx * cos - vy * sin), getY() + (vx * sin + vy * cos));
  }

  return res;
}

Obb::Obb(const std::array<VectorD, 4> &vertices) {
  halfLength = (vertices[2] - vertices[1]).getLength() / 2;
  halfWidth = (vertices[1] - vertices[0]).getLength() / 2;
  position = (vertices[0] + vertices[2]) / 2;
  auto direction = normalize(vertices[1] - vertices[2]);
  double cosTheta = dot(direction, VectorD(1, 0));
  if (direction.getY() >= 0)
    rotation = std::acos(cosTheta);
  else
    rotation = 2 * Pi - std::acos(cosTheta);
}

} // namespace ds
