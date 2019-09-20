#ifndef DRIVE_SINGULARITY_MATH_VECTOR_H
#define DRIVE_SINGULARITY_MATH_VECTOR_H

#include <cassert>
#include <cmath>

namespace ds {

class VectorD {
public:
  VectorD() = default;
  VectorD(double x, double y) : x(x), y(y) {}

  const double getX() const { return x; };

  double &getX() { return x; }

  const double getY() const { return y; }

  double &getY() {return y; }

  const double getLength() const { return std::sqrt(x * x + y * y); }

  const double getRotation() const { return std::atan2(y, x); }

  // TODO: fast inverse square root ?

private:
  double x = 0, y = 0;
};

inline VectorD operator*(const VectorD &u, double scalar) {
  return {u.getX() * scalar, u.getY() * scalar};
}

inline VectorD operator/(const VectorD &u, double scalar) {
  return u * (1 / scalar);
}

inline VectorD operator-(const VectorD &u, const VectorD &v) {
  return {u.getX() - v.getX(), u.getY() - v.getY()};
}

inline VectorD operator+(const VectorD &u, const VectorD &v) {
  return {u.getX() + v.getX(), u.getY() + v.getY()};
}

// Compute the dot product of two vectors
inline double dot(const VectorD &u, const VectorD &v) {
  return u.getX() * v.getX() + u.getY() * v.getY();
}

inline double project(const VectorD &u, const VectorD &v) {
	return dot(u, v) / v.getLength();
}

inline double cos(const VectorD &u, const VectorD &v) {
	return dot(u, v) / (v.getLength() * u.getLength());
}

inline double cross(const VectorD &u, const VectorD &v) {
  return u.getX() * v.getY() - u.getY() * v.getX();
}

inline VectorD normalize(const VectorD &u) { return u / u.getLength(); }

inline VectorD rotate90(const VectorD &u) { return {u.getY(), -u.getX()}; }

inline VectorD rotate(const VectorD & u, double radian) {
  auto c = std::cos(radian), s = std::sin(radian);
  auto x = u.getX(), y = u.getY();
  return {c * x - s * y, s * x + c * y};
}

inline VectorD rotate(const VectorD &u, const VectorD &v) {
  return {cross(u, v), dot(u, v)};
}

} // namespace ds

#endif // DRIVE_SINGULARITY_MATH_VECTOR_H
