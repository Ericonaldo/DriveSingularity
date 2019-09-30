//
// Created by ming on 7/30/19.
//

#ifndef DRIVE_SINGULARITY_TMP_CONTROL_H
#define DRIVE_SINGULARITY_TMP_CONTROL_H

#include "DriveSingularity/Utils/Events.h"
#include "DriveSingularity/Vehicle/Vehicle.h"

#include <iostream>
#include <utility>
#include <unordered_set>

namespace ds {
namespace control {

enum VehicleType { Agent = 0, Social };

/* controlling parameters */
constexpr double MaxVelocity = 200;
constexpr double MaxAcceleration = 35;
constexpr double MaxComfortableAcceleration = 15;
constexpr double MinComfortableAcceleration = -30;
constexpr double Delta = 4;
constexpr double DistanceWanted = 30;
constexpr double TimeWanted = 1.5;
constexpr double MaxSteeringAngle = Pi / 3;
constexpr double TauAccel = 0.6;
constexpr double TauDS = 0.2;
constexpr double KpAccel = 1 / TauAccel;
constexpr double KpHeading = 1 / TauDS;
constexpr double KpLateral = 0.5;
constexpr double LaneChangeDelay = 5;
constexpr double LaneChangeMaxBrakingImposed = 10;
constexpr double Politeness = 0.5;
constexpr double LaneChangeMinAccelGain = 5;

constexpr double CollisionReward = -1;
constexpr double HighVelocityReward = 0.4;
constexpr double LaneChangeReward = 0;

static VehicleId VEHICLE_ID_MANAGER = 0;

class VehicleController
    : public VehicleEntity,
      public std::enable_shared_from_this<VehicleController> {
public:
  
  VehicleController(std::shared_ptr<roadmap::RoadMap> rmap,
                    std::shared_ptr<roadmap::Graph> rgraph, double halfLength,
                    double halfWidth, double x, double y, double rotation,
                    double velocity, double targetVelocity,
                    roadmap::LaneId laneId, roadmap::LaneId targetLaneId)
      : VehicleEntity(halfLength, halfWidth, x, y, rotation, velocity,
                      targetVelocity, laneId, targetLaneId),
        roadMap(std::move(rmap)), roadGraph(std::move(rgraph)) {
    id = VEHICLE_ID_MANAGER++;
  }

  ~VehicleController() override = default;

  VehicleId getId() const { return id; }

  double getTimer() const { return timer; }
  double &getTimer() { return timer; }

  void setTimer(double v) { timer = v; }

  void controlSteering();
  void followRoad();

  virtual double calculateReward() const { return 0; }

  void setOutMap(bool val) { outMap = val; }
  void setCrash(bool val) { crashed = val; }

  bool isCrashed() const { return crashed; }
  bool isOutMap() const { return outMap; }
  bool isTerminate() const { return outMap || crashed; }

  VehicleType getVehicleType() const { return vehicleType; }
  void setVehicleType(VehicleType type) { vehicleType = type; }

public: // !!! you must implement them !!!
  virtual void pickLane() = 0;
  virtual void controlAccelerate() = 0;

  virtual void step(double dt) = 0;
  virtual double calculateAcc(const std::shared_ptr<VehicleController>&, bool) = 0;
  virtual double getDesiredGap(const std::shared_ptr<VehicleController>&) = 0;

public: // API
	// TODO(ming): support multiple events.
  void setEventListening(EventFlag::Type event) { eventListening.emplace(event); }
//  EventFlag::Type getEventListening() const { return eventListening; }
  std::unordered_set<EventFlag::Type> getEventListening() const { return eventListening; }

private:
  VehicleId id;
  double timer = 0;
  bool crashed = false;
  bool outMap = false;
  VehicleType vehicleType;
//  EventFlag::Type eventListening = EventFlag::None;
  std::unordered_set<EventFlag::Type> eventListening;

protected:
  std::shared_ptr<roadmap::RoadMap> roadMap;
  std::shared_ptr<roadmap::Graph> roadGraph;
};

} // namespace control
} // namespace ds

#endif // DRIVE_SINGULARITY_TMP_CONTROL_H
