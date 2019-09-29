//
// Created by ming on 7/30/19.
//

#ifndef DRIVE_SINGULARITY_IDMVEHICLE_H
#define DRIVE_SINGULARITY_IDMVEHICLE_H

#include "DriveSingularity/Control/Control.h"

namespace ds {
namespace control {
class IDMVehicle : public VehicleController {
public:
  IDMVehicle(const std::shared_ptr<roadmap::RoadMap> &rmap,
             const std::shared_ptr<roadmap::Graph> &rgraph, double halfLength,
             double halfWidth, double x, double y, double rotation,
             double velocity, double targetVelocity, roadmap::LaneId laneId,
             roadmap::LaneId targetLaneId)
      : VehicleController(rmap, rgraph, halfLength, halfWidth, x, y, rotation,
                          velocity, targetVelocity, laneId, targetLaneId) {
    setVehicleType(VehicleType::Social);
  }

  void pickLane() override;
  void controlAccelerate() override;

  void step(double dt) override;

  ~IDMVehicle() override = default;

private:
  bool mobil(roadmap::LaneId candidateLaneId);

  double getDesiredGap(const std::shared_ptr<VehicleController> &frontVehicle) override;
  double calculateAcc(const std::shared_ptr<VehicleController> &vehicle, bool front) override;
  std::pair<double, VehicleId>
  getFirstVPair(std::shared_ptr<roadmap::StraightRoad> &road,
                bool reversed) const;

  std::shared_ptr<VehicleController> getFrontVehicle() const;
  std::shared_ptr<VehicleController> getFrontVehicle(const roadmap::LaneId &laneId,
                                              bool switchLane = false) const;
  std::shared_ptr<VehicleController> getRearVehicle(roadmap::LaneId laneId) const;

  VectorD getBarrierPositionOnStraightRoad();

private:
  double accDecay(const VectorD & barrierPos);
};

} // namespace control
} // namespace ds

#endif // DRIVE_SINGULARITY_IDMVEHICLE_H
