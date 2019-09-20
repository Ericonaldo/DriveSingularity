// This file contains functions for collision detection.
// Note that the results may not be accurate due to the round off
// error. Meanwhile, the colinear cases are although ignored.
// This inaccuracy will cause no problem in our collision detection
// scenario.

#ifndef DRIVE_SINGULARITY_MATH_COLLISION_H
#define DRIVE_SINGULARITY_MATH_COLLISION_H

#include "DriveSingularity/Math/Obb.h"
#include "DriveSingularity/Math/Segment.h"

namespace ds {

bool checkCollision(const Obb &obb1, const Obb &obb2);

bool checkCollision(const Segment &seg1, const Segment &seg2);

bool checkCollision(const Obb &obb, const Segment &seg);

bool checkCollision(const Segment &seg, const Obb &obb);

} // namespace ds

#endif // DRIVE_SINGULARITY_MATH_COLLISION_H
