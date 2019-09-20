#ifndef DRIVE_SINGULARITY_AGENTVEHICLE_H
#define DRIVE_SINGULARITY_AGENTVEHICLE_H

#include <memory>

#include "DriveSingularity/Control/Action.h"
#include "DriveSingularity/Control/Control.h"
#include "DriveSingularity/RoadMap/Graph.h"
#include "DriveSingularity/RoadMap/RoadMap.h"

namespace ds {
namespace control {

class AgentVehicle : public VehicleController {
public:
  AgentVehicle(const std::shared_ptr<roadmap::RoadMap> &rmap,
               const std::shared_ptr<roadmap::Graph> &rgraph, double halfLength,
               double halfWidth, double x, double y, double rotation,
               double velocity, double targetVelocity, roadmap::LaneId laneId,
               roadmap::LaneId targetLaneId)
      : VehicleController(rmap, rgraph, halfLength, halfWidth, x, y, rotation,
                          velocity, targetVelocity, laneId, targetLaneId) {
    setVehicleType(VehicleType::Agent);
    action = discrete::Action::Idle;
  }

  void step(double dt) override;
  void pickLane() override;
  void controlAccelerate() override;

  double calculateAcc(const std::shared_ptr<VehicleController>&, bool) override { return getAcceleration(); }
  double getDesiredGap(const std::shared_ptr<VehicleController>& frontVehicle) override {
	  double d0 =
			  control::DistanceWanted + getHalfLength() + frontVehicle->getHalfLength();
	  double tau = control::TimeWanted;
	  double ab = -control::MinComfortableAcceleration * MaxComfortableAcceleration;
	  double dv = getVelocity() - frontVehicle->getVelocity();
	  return d0 + getVelocity() * tau + getVelocity() * dv / (2 * std::sqrt(ab));
  }

  double calculateReward() const override;

  void act();

  void setAction(discrete::Action action);

public: // API
        //  std::unordered_map<event::EVENT_TYPE, float>
        //  retrieveEventEvaluation() override;
private:
  discrete::Action action;
  bool actionExecuted = false;

  static constexpr double DeltaVelocity = 5.0;
};

} // namespace control
} // namespace ds

#endif // DRIVE_SINGULARITY_AGENTVEHICLE_H
