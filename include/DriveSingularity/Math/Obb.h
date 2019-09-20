// Oriented Bounded Box

#ifndef DRIVE_SINGULARITY_MATH_OBB_H
#define DRIVE_SINGULARITY_MATH_OBB_H

#include <array>
#include <cmath>

#include "DriveSingularity/Math/Vector.h"

namespace ds {

class Obb {
public:
  Obb(double halfLength, double halfWidth, VectorD position,
      double rotation = 0)
      : halfLength(halfLength), halfWidth(halfWidth), position(position),
        rotation(rotation) {}

  Obb(double halfLength, double halfWidth, double x, double y,
      double rotation = 0)
      : halfLength(halfLength), halfWidth(halfWidth), position(x, y),
        rotation(rotation) {}

  explicit Obb(const std::array<VectorD, 4> &vertices);

  virtual ~Obb() = default;

  const double getX() const { return position.getX(); }

  double &getX() { return position.getX(); }

  const double getY() const { return position.getY(); }

  double &getY() { return position.getY(); }

  const VectorD getPosition() const { return position; }

  VectorD &getPosition() { return position; }

  const double getHalfLength() const { return halfLength; }

  const double getHalfWidth() const { return halfWidth; }

  const double getRotation() const { return rotation; }

  double &getRotation() { return rotation; }

  bool inBox(const VectorD& pos) const {
	  const VectorD& center = getPosition();
	  double dis = (center - pos).getLength();
	  return dis <= getHalfLength();
  }

  const VectorD getLenVec() const {
    return VectorD(halfLength * std::cos(rotation),
                   halfWidth * std::sin(rotation));
  }

  const VectorD getWidVec() const {
    static const double HalfPi = std::acos(0);
    return VectorD(halfWidth * std::cos(rotation + HalfPi),
                   halfWidth * std::sin(rotation + HalfPi));
  }

  std::array<VectorD, 4> getVertices() const;

private:
  double halfLength = 0, halfWidth = 0;
  VectorD position;
  double rotation = 0;

  // TODO: maybe we should cache cos and sin
};

} // namespace ds

#endif // DRIVE_SINGULARITY_ENGINE_MATH_OBB_H
