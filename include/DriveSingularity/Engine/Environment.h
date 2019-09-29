#ifndef DRIVE_SINGULARITY_ENGINE_ENVIRONMENT_H
#define DRIVE_SINGULARITY_ENGINE_ENVIRONMENT_H

#include <list>
#include <map>
#include <memory>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

#include "DriveSingularity/Control/Action.h"
#include "DriveSingularity/Control/AgentVehicle.h"
#include "DriveSingularity/Control/Control.h"
#include "DriveSingularity/Math/Collision.h"
#include "DriveSingularity/RoadMap/RoadMap.h"
#include "DriveSingularity/Utils/Events.h"
#include "DriveSingularity/Utils/Logger.h"
#include "DriveSingularity/Utils/VehicleGenerator.h"
#include "DriveSingularity/Vehicle/Common.h"

namespace ds {
namespace engine {

using namespace utils;

typedef std::array<std::vector<std::vector<double>>, 3> Observation;

struct FeedBack {
  VehicleId id;
  Observation observation;
  double reward;
  bool done;
  std::vector<EventContainer> events;

  FeedBack(VehicleId id, Observation observation, double reward, bool done,
           std::vector<EventContainer> events)
      : id(id), observation(std::move(observation)), reward(reward), done(done),
        events(std::move(events)) {}
  FeedBack(VehicleId id, Observation observation, double reward, bool done)
      : FeedBack(id, std::move(observation), reward, done, {}) {}
};


class Environment {
public: // API
  Environment(const roadmap::RoadMap& roadMap, const roadmap::Graph& roadGraph)
      : roadMap(std::make_shared<roadmap::RoadMap>(roadMap)),
        roadGraph(std::make_shared<roadmap::Graph>(roadGraph)) {
      static VectorD videoAnchor1(0, 0);
      static VectorD videoAnchor2(3000, 2400);
      bool flag = true;
      for (const auto &roadpair : roadMap.getRoads()) {
        const auto & road = roadpair.second;
        if (road->getRoadType() == roadmap::RoadType::Straight) {
          auto straightRoad = std::dynamic_pointer_cast<roadmap::StraightRoad>(road);
          if (flag) {
            setAnchor1(straightRoad->getFromBorder().getEnd());
            flag = false;
          }
          updateAnchor1(straightRoad->getFromBorder().getEnd());
          updateAnchor1(straightRoad->getFromBorder().getStart());
          updateAnchor1(straightRoad->getToBorder().getEnd());
          updateAnchor1(straightRoad->getToBorder().getStart());
          updateAnchor2(straightRoad->getFromBorder().getEnd());
          updateAnchor2(straightRoad->getFromBorder().getStart());
          updateAnchor2(straightRoad->getToBorder().getEnd());
          updateAnchor2(straightRoad->getToBorder().getStart());
          for (size_t k = 0; k < straightRoad->getLanes().size() ; ++k) {
            const auto &lane = straightRoad->getLanes().at(k);
            auto axi = straightRoad->getLaneAxis(k);
            VectorD position;
            double rotation;
            if (lane.isReversed()) {
              position = axi.getEnd();
              rotation = (axi.getStart() - axi.getEnd()).getRotation();
            } else {
              position = axi.getStart();
              rotation = (axi.getEnd() - axi.getStart()).getRotation();
            }
            //if (position.getX() < videoAnchor1.getX() || position.getY() < videoAnchor1.getY() ||
              //  position.getX() > videoAnchor2.getX() || position.getY() > videoAnchor2.getY()) {
              generators.emplace_back(std::make_pair(position, rotation));
            //}
          }
        }
      }

    logger = Logger(); // create logger
    agentActionType = control::ACTION_SPACE_TYPE::DISCRETE;  // default action space is discrete
  }
  
  void setLogger(std::string& fpath) { logger.setFilePath(fpath);  logger.clear(); }
  void record() { logger.log(epoch, vehicles); logger.log(roadMap->getNodes()); }
  void saveRecord() { logger.writeToFile(); }
  void turnOnRecord() { recordOn = true; }
  void turnOffRecord() { recordOn = false; }

  void loadVehicles(std::list<std::unordered_map<std::string, float>> &vehicles_config);
	void initGenerator(size_t socialVehicleNum, std::pair<double, double> halfLengthRange,
			std::pair<double, double> halfWidthRange, std::pair<double, double> velocityRange, std::pair<double, double> targetVelocityRange) {
		setMaxVehicle(socialVehicleNum);
		vehicleGenerator.setHalfLengthRange(halfLengthRange);
		vehicleGenerator.setHalfWidthRange(halfWidthRange);
		vehicleGenerator.setVelocityRange(velocityRange);
		vehicleGenerator.setTargetVelocityRange(targetVelocityRange);
	}

  int loadGenerators(
    const std::list<std::unordered_map<std::string, float>> &generator_config);

	void generateVehicles();

  template <class VehicleType>
  VehicleId addVehicle(double halfLength, double halfWidth, double x, double y,
                       double rotation, double velocity, double targetVelocity);

  EventContainer evaluate(VehicleId vehicleId) const;

  // TODO(ming): bubble search.
  Observation getObservation(VehicleId vehicleId, std::size_t viewLength = 84,
                             std::size_t viewWidth = 84);

  void reset() {
//    roadMap->reset();
//    roadGraph->reset();
    vehicles.clear();
    vehicleGrid.clear();
    vehicleGridPos.clear();
    eraseVehicleId.clear();
    agents.clear();

    for (auto& kv: roadMap->getRoads()) kv.second->clear();
    for (auto& kv: roadMap->getNodes()) kv.second->clear();

    control::VEHICLE_ID_MANAGER = 0;

    epoch = 0;
  }

  void addListener(VehicleId vehicleId, std::vector<EventFlag::Type> events) {
    for (auto e : events) {
      vehicles.at(vehicleId)->setEventListening(e);
//      std::cout << "[DEBUG] set event: " << e << std::endl;
    }
  }

  std::vector<FeedBack>
  step(const std::unordered_map<VehicleId, std::size_t> &action);

//  std::unordered_map<VehicleId, std::pair<int, int>> observation_space;
  std::unordered_map<VehicleId, int> action_space;

public: // c++ public callable
  static Environment makeHighway();

  bool step();

  const std::unordered_map<VehicleId,
                           std::shared_ptr<control::VehicleController>> &
  getVehicles() const {
    return vehicles;
  }

  std::unordered_set<VehicleId> getAgents() { return agents; }

  const roadmap::RoadMap &getRoadMap() const { return *roadMap; }
  const roadmap::Graph &getGraph() const { return *roadGraph; }

  template <class VehicleType>
  VehicleId addVehicle(const std::unordered_map<std::string, float> &attrs);

  VehicleId addVehicle(std::shared_ptr<control::VehicleController> vehicle);

public: // inner callable
  /*
   * get grid coordinate
   */
  std::pair<int, int> getGridPos(const VectorD &position) {
    return std::make_pair(int(position.getX() / cellX),
                          int(position.getY() / cellY));
  }

  /**
   * check if a segment intersects with a cell
   */
  bool isIntersect(const std::pair<int, int> &gridPos, const Segment &segment);

  /**
   * check if a position covered by a cell
   */
  bool isCover(const std::pair<int, int> &gridPos, const VectorD &position);

  void setAnchor1(const VectorD &anchor1);

  void setAnchor2(const VectorD &anchor2);

  void updateAnchor1(const VectorD &anchor1);

  void updateAnchor2(const VectorD &anchor2);

  VehicleGenerator &getVehicleGenerator(){
    return vehicleGenerator;
  }

private:
  bool checkCollision(VehicleId vehicleId) const;

  bool checkOutMap(VehicleId vehicleId) const;

  void roadMapChecking();

  void updateOnrampVehicleCounter(std::shared_ptr<roadmap::Onramp> &onramp);

  void updateVehicleMap();

  bool isTerminal() const;

  control::discrete::Action getAction(std::size_t id);

  /**
   * Find vehicles in a certain rectangle
   *
   * @param center the center of rectangle
   * @param rangeLength the length of rectangle
   * @param rangeWidth the width of rectangle
   *
   * @return a list of vehicles
   */
  std::list<std::shared_ptr<control::VehicleController>>
  getAdjacentVehicles(const VectorD &center, double rangeLength,
                      double rangeWidth);

  /**
   * Find road lines that intersect with WINDOW
   *
   * @param window
   * @return list<pair<road line, whether the line is continuous>>
   */
  std::list<std::pair<Segment, bool>>
  getInvolvedRoadLines(const Obb &window);


public:
  void setMaxVehicle(size_t maxVehicle);

private:
  bool recordOn = false;
  std::size_t epoch = 0;
  std::list<size_t> eraseVehicleId;

  VehicleGenerator vehicleGenerator;
  std::unordered_map<VehicleId, std::shared_ptr<control::VehicleController>>
      vehicles;
  std::map<std::pair<int, int>, std::unordered_set<VehicleId>> vehicleGrid;
  std::map<VehicleId, std::pair<int, int>> vehicleGridPos;
  std::unordered_set<VehicleId> agents;

  std::shared_ptr<roadmap::RoadMap> roadMap;
  std::shared_ptr<roadmap::Graph> roadGraph;
  VectorD anchor1, anchor2;
  size_t maxVehicle;

  std::vector<std::pair<VectorD, double>> generators;

  // TODO: Maintain a (coarse) mesh so that we can find the objects near
  //    a specific vehicle without iterating all objects.

  std::unordered_map<VehicleId, std::vector<EventContainer>> events;
  Logger logger;

  control::ACTION_SPACE_TYPE agentActionType;

private: // predefined simulation attributions
  static constexpr double cellX = 100, cellY = 100;
  static constexpr std::size_t SimulationFrequency = 15;
  static constexpr std::size_t OperationInterval = 15;
  static constexpr double dt = 1.0 / SimulationFrequency;
  static constexpr std::size_t MaxEpoch = 5000;

public:
  static constexpr std::size_t Magnification =
      5; // the ratio of rangeLength(rangeWidth) to viewLength(viewWidth)
};

} // namespace engine
} // namespace ds

namespace ds {
namespace engine {

template <class VehicleType>
VehicleId Environment::addVehicle(double halfLength, double halfWidth, double x,
                                  double y, double rotation, double velocity,
                                  double targetVelocity) {
  auto laneId = roadMap->locate(VectorD(x, y));
  // TODO(ming): customize target lane.
  auto targetLaneId = laneId;
  auto vehicle = std::make_shared<VehicleType>(
      roadMap, roadGraph, halfLength, halfWidth, x, y, rotation, velocity,
      targetVelocity, laneId, targetLaneId);
  auto vc = std::dynamic_pointer_cast<control::VehicleController>(vehicle);
  VehicleId id = addVehicle(vc);
  if (std::dynamic_pointer_cast<control::AgentVehicle>(vehicles.at(id))) {
    agents.insert(id);
  }
  return id;
}

template <class VehicleType>
VehicleId
Environment::addVehicle(const std::unordered_map<std::string, float> &attrs) {
  auto laneId = roadMap->locate(VectorD(attrs.at("x"), attrs.at("y")));
  auto targetLaneId = laneId;
  auto vehicle = std::make_shared<VehicleType>(
      roadMap, roadGraph, attrs.at("halfLength"), attrs.at("halfWidth"),
      attrs.at("x"), attrs.at("y"), attrs.at("rotation"), attrs.at("velocity"),
      attrs.at("targetVelocity"), laneId, targetLaneId);
  auto vc = std::dynamic_pointer_cast<control::VehicleController>(vehicle);
  VehicleId id = addVehicle(vc);
  if (std::dynamic_pointer_cast<control::AgentVehicle>(vehicles.at(id))) {
    agents.insert(id);
  }
  return id;
}

} // namespace engine
} // namespace ds

namespace ds {
namespace engine {

Observation drawObservation(
    const VectorD &center, std::size_t rangeLength, std::size_t rangeWidth,
    const std::list<std::shared_ptr<control::VehicleController>>
        &adjacentVehicles,
    const std::list<std::pair<Segment, bool>> &involvedLines);

void printObservation(const Observation &observation);

} // namespace engine
} // namespace ds

#endif // DRIVE_SINGULARITY_ENGINE_ENVIRONMENT_H
