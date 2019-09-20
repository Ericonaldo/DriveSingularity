#include <utility>

#ifndef DRIVE_SINGULARITY_ENGINE_VEHICLE_H
#define DRIVE_SINGULARITY_ENGINE_VEHICLE_H

#include <cstddef>
#include <functional>
#include <json/json.h>
#include <limits>
#include <list>
#include <memory>

#include "DriveSingularity/Math/Constants.h"
#include "DriveSingularity/Math/Obb.h"
#include "DriveSingularity/RoadMap/Graph.h"
#include "DriveSingularity/RoadMap/RoadMap.h"
#include "DriveSingularity/Vehicle/Common.h"

namespace ds {

/**
 * Vehicle entity
 */
class VehicleEntity : public Obb {
public:
  VehicleEntity(double halfLength, double halfWidth, double x, double y,
                double rotation, double velocity, double targetVelocity,
                roadmap::LaneId laneId, roadmap::LaneId targetLaneId)
      : Obb(halfLength, halfWidth, x, y, rotation), velocity(velocity),
        targetVelocity(targetVelocity), laneId(std::move(laneId)),
        targetLaneId(std::move(targetLaneId)) {
    //		atNode = nullptr;
    lastLaneId = this->laneId;
  }

  virtual ~VehicleEntity() = default;

public:
  double getVelocity() const { return velocity; }
  void setVelocity(double v) { velocity = v; }

  double getTargetVelocity() const { return targetVelocity; }
  void setTargetVelocity(double v) { targetVelocity = v; }

  double getAcceleration() const { return acceleration; }
  void setAcceleration(double v) { acceleration = v; }

  double getSteering() const { return steering; }
  void setSteering(double v) { steering = v; }

  roadmap::LaneId getLastLaneId() const { return lastLaneId; }
  void setLastLaneId(const roadmap::LaneId &laneId1) { lastLaneId = laneId1; }

  roadmap::LaneId getLaneId() const { return laneId; }
  roadmap::LaneId &getLaneId() { return laneId; }
  void setLaneId(const roadmap::LaneId &laneId1) { laneId = laneId1; }

  roadmap::LaneId getTargetLaneId() const { return targetLaneId; }
  roadmap::LaneId &getTargetLaneId() { return targetLaneId; }
  void setTargetLaneId(const roadmap::LaneId &laneId1) {
    targetLaneId = laneId1;
  }

  double getMinSafeDistance() const { return minSafeDistance; }
  double getLimitedDistance() const { return safeDistance; }
  double getWantedSafeDistance() const {
    return std::max(velocity * velocity / 254 / 0.7, minSafeDistance);
  }

  void setSafeDistance(double v) { safeDistance = v; }
  void setMinSafeDistance(double v) { minSafeDistance = v; }

  const std::list<roadmap::LaneId> &getRoute() const { return route; }
  std::list<roadmap::LaneId> &getRoute() { return route; }

  bool isTurnRight() const { return rightTurnLight; }
  bool isTurnLeft() const { return leftTurnLight; }
  bool isDecelerate() const { return decelerateLight; }

  void updateLight(double eps) {
    leftTurnLight = steering > eps;
    rightTurnLight = steering < -eps;
    decelerateLight = acceleration < -eps;
  }

  void popRoute() {
    if (!route.empty())
      route.pop_front();
  }

  Json::Value retrieveInfo(bool flag, size_t timestep);

private:
  double velocity;
  double targetVelocity;
  double acceleration = 0;
  double steering = 0;
  double safeDistance = 30;
  double minSafeDistance = 30;

  roadmap::LaneId lastLaneId;
  roadmap::LaneId laneId;
  roadmap::LaneId targetLaneId;
  roadmap::LaneId nextLaneId;

  std::list<roadmap::LaneId> route;

  bool leftTurnLight = false;
  bool rightTurnLight = false;
  bool decelerateLight = false;

protected:
  bool gotFirstTargetLane = false;
};

} // namespace ds

#endif // DRIVE_SINGULARITY_ENGINE_VEHICLE_H
