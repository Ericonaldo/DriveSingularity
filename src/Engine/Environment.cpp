#include "DriveSingularity/Engine/Environment.h"
#include "DriveSingularity/Control/AgentVehicle.h"
#include "DriveSingularity/Control/Control.h"
#include "DriveSingularity/Control/simple/IDMVehicle.h"
#include "DriveSingularity/Math/Collision.h"
#include "DriveSingularity/RoadMap/RoadMapBuilder.h"
#include "DriveSingularity/RoadMap/Roads.h"
#include <array>
#include <iostream>
#include <utility>
#include <algorithm>
#include <climits>

namespace ds {
namespace engine {

int Environment::loadGenerators(
    const std::list<std::unordered_map<std::string, float>> &generator_config) {
  // TODO(ming): load generator from configurations
  return 0;
}

void Environment::generateVehicles() {
  if (vehicles.size() >= maxVehicle) return;
// random add vehicles
    static std::mt19937_64 eng(std::random_device{}());
    std::shuffle(generators.begin(), generators.end(), eng);
    for (const auto &generatorPoint: generators) {
      const auto &position = generatorPoint.first;
      double rotation = generatorPoint.second;
      vehicleGenerator.gen();
      double safeDis = std::max(vehicleGenerator.halfLength, vehicleGenerator.halfWidth)* 10;
      auto adjacentVehicles = getAdjacentVehicles(position, safeDis, safeDis);
      if (adjacentVehicles.empty()) {
        addVehicle<control::IDMVehicle>(
                vehicleGenerator.halfLength, vehicleGenerator.halfWidth, position.getX(),
                position.getY(), rotation, vehicleGenerator.velocity,
                vehicleGenerator.targetVelocity);
      }
      if (vehicles.size() == maxVehicle) break;
    }
}

/**
 * Add a vehicle instance.
 *
 * @param vehicle Vehicle, an instance.
 */
VehicleId
Environment::addVehicle(std::shared_ptr<control::VehicleController> vehicle) {
  auto id = vehicle->getId();
  auto laneId = vehicle->getLaneId();
  auto &road = roadMap->getRoads().at(laneId.first);
  road->addVehicle(vehicle->getId(), vehicle);
  vehicleGridPos[id] = getGridPos(vehicle->getPosition());
  vehicleGrid[vehicleGridPos[id]].emplace(vehicle->getId());
  vehicles.emplace(id, std::move(vehicle));
  return id;
}

/**
 * Checking.
 */
void Environment::roadMapChecking() {
  auto vertexes = roadGraph->getVertexes();

  for (const auto &vertex : vertexes) {
    auto meta = vertex.second;
    if (meta.nodeType == roadmap::NodeType::Crossroads) {
      // TODO(yifan): update traffic lights for the crossroads
      auto crossroads = roadMap->getRoad<roadmap::Crossroads>(meta.nodeId);
      crossroads->updateTrafficLights();
    } else if (meta.nodeType == roadmap::NodeType::Onramp) {
      auto onramp = roadMap->getRoad<roadmap::Onramp>(meta.nodeId);
      updateOnrampVehicleCounter(onramp);
    } else if (meta.nodeType == roadmap::NodeType::Offramp) {
      // do nothing
    } else if (meta.nodeType == roadmap::NodeType::Terminal) {
    }
  }
}

void Environment::updateOnrampVehicleCounter(
    std::shared_ptr<roadmap::Onramp> &onramp) {
  auto onrampVehicles = onramp->getVehicles();
  auto linkedRoads = onramp->getLinkedRoads();
  int v0 = 0, v1 = 0;
  for (auto &kv : onrampVehicles) {
    auto lastLane = kv.second->getLastLaneId();
    if (lastLane.first == linkedRoads[0]) {
      std::size_t lane;
      auto road = roadMap->getRoad<roadmap::StraightRoad>(lastLane.first);
      assert(road);
      if (roadGraph->getEdges().at(road->getId()).to == onramp->getId())
        lane = road->getLanes().size() - 1;
      else
        lane = 0;
      if (lastLane.second == lane)
        ++v0;
    } else if (lastLane.first == linkedRoads[1]) {
      // Assume that the merged road has only one lane
      ++v1;
    }
  }
  onramp->setCounter(v0, v1);
}

/**
 * Update vehicle map.
 *
 * Move vehicles from roads to roads.
 */
void Environment::updateVehicleMap() {
  for (auto &kv : vehicles) {
    // remove from old road
    auto v = kv.second;

    roadmap::LaneId oldLane = v->getLastLaneId();
    roadmap::LaneId curLane = v->getLaneId();

	  if (oldLane == curLane && v->isTerminate()) {
		  roadMap->getRoads().at(oldLane.first)->removeVehicle(kv.first);
		  continue;
	  }

    auto oldRoad = roadMap->getRoads().at(oldLane.first);

    oldRoad->removeVehicle(kv.first);

    auto curRoad = roadMap->getRoads().at(curLane.first);
    assert(curRoad);
    if (v->isTerminate()) {
      curRoad->removeVehicle(kv.first);
    } else {
      if (curRoad->getNodeType() == roadmap::NodeType::Terminal) continue;
      curRoad->addVehicle(kv.first, kv.second);
    }
  }
}

bool Environment::isTerminal() const {
  if (epoch > MaxEpoch)
    return true;

  for (auto id: agents) {
      const auto& agent = vehicles.at(id);
      if (agent->isTerminate()) return true;
  }

  return false;
}

control::discrete::Action Environment::getAction(std::size_t id) {
  static std::array<control::discrete::Action, 5> actions = {
      control::discrete::Idle, control::discrete::Faster,
      control::discrete::Slower, control::discrete::TurnLeft,
      control::discrete::TurnRight};
  return actions[id];
}

/**
 * State transition simulation.
 *
 * This method will step the environment with state transition ...
 *
 * @return (refactor) done or not.
 */
bool Environment::step() {
  ++epoch;

  if (recordOn) {
      logger.log(epoch, vehicles);
      logger.log(roadMap->getRoads());
  }

  // intersection control
  roadMapChecking(); // call manage

  // vehicle control
  for (const auto &kv : vehicles) {
	  auto id = kv.first;
	  auto &vehicle = kv.second;
	  //    assert(vehicle);
	  auto oldGridPos = vehicleGridPos[id];
	  // vehicles execute and reward update

//    std::cout << "[DEBUG] 0" << std::endl;
//	  std::cout << "[DEBUG] >>>>>>>>>>>> add social" << std::endl;
	  vehicle->step(dt);
//    std::cout << "[DEBUG] 1" << std::endl;
//	  std::cout << "[DEBUG] <<<<<<<<<<< finish" << std::endl;

	  auto newGridPos = getGridPos(vehicle->getPosition());

	  // remove vehicle on borderline.
	  auto lane = vehicle->getTargetLaneId();
	  auto &road = roadMap->getRoads().at(lane.first);

	  vehicle->setOutMap(checkOutMap(id));
//    std::cout << "[DEBUG] 2" << std::endl;
	  if (road->getNodeType() == roadmap::NodeType::Terminal ||
	      vehicle->isOutMap()) {
		  if (vehicle->getVehicleType() != control::VehicleType::Agent) {
			  eraseVehicleId.emplace_back(id);
		  }
		  vehicleGrid[oldGridPos].erase(id);
		  vehicleGridPos.erase(id);
	  } else if (oldGridPos != newGridPos) {
		  vehicleGrid[oldGridPos].erase(id);
		  vehicleGridPos[id] = newGridPos;
		  vehicleGrid[newGridPos].emplace(id);
	  }
//    std::cout << "[DEBUG] 3" << std::endl;

  }

  for (const auto &kv: vehicles) {
    auto id = kv.first;
    const auto &v = kv.second;
    v->setCrash(checkCollision(id));
    if (v->isCrashed() && v->getVehicleType() != control::VehicleType::Agent) {
        vehicleGrid[vehicleGridPos[id]].erase(id);
        vehicleGridPos.erase(id);
        eraseVehicleId.emplace_back(id);
    }
  }

//  std::cout << "[DEBUG] second loop" << std::endl;

    // update vehicle map
    updateVehicleMap();

	for (auto k : eraseVehicleId) {
		vehicles.erase(k);
	}


//  std::cout << "[DEBUG] third loop" << std::endl;

  for (auto id : agents) {
    events[id].emplace_back(evaluate(id));
  }

//  std::cout << "[DEBUG] fourth loop" << std::endl;

  eraseVehicleId.clear();

  return true;
}

std::vector<FeedBack>
Environment::step(const std::unordered_map<VehicleId, std::size_t> &action) {

  for (const auto id : agents) {
    auto vehicle = std::dynamic_pointer_cast<control::AgentVehicle>(vehicles.at(id));
    vehicle->setAction(getAction(action.at(id)));
  }

  events.clear();

  for (size_t t = 0; t < OperationInterval; ++t) {
    step();
    if (isTerminal()) break;
  }

  std::vector<FeedBack> feedback;

  for (auto &id: agents) {
  	auto& vehicle = vehicles.at(id);

    feedback.emplace_back(id, getObservation(id),
                          vehicle->calculateReward(), vehicle->isTerminate(),
                          events.at(id));

	  if (vehicle->isTerminate()) {
	  	eraseVehicleId.emplace_back(id);
	  }
  }

  generateVehicles();

  updateVehicleMap();

  // remove dead agents (collision)
  for (auto k : eraseVehicleId) {
      vehicleGrid[vehicleGridPos[k]].erase(k);
      vehicleGridPos.erase(k);
      vehicles.erase(k);
      agents.erase(k);
  }

  eraseVehicleId.clear();

  return feedback;
}

Observation Environment::getObservation(VehicleId vehicleId,
                                        std::size_t viewLength,
                                        std::size_t viewWidth) {
  const auto &vehicle = vehicles.at(vehicleId);
  assert(vehicle);
  auto center = vehicle->getPosition();

  std::size_t rangeLength = viewLength * Magnification,
              rangeWidth = viewWidth * Magnification;

//    std::cout << "[DEBUG] ready to get vehicle observation" << std::endl;
    auto adjacentVehicles =
      getAdjacentVehicles(center, rangeLength, rangeWidth);
//    std::cout << "[DEBUG] got observation" << std::endl;
    auto involvedRoadLines = getInvolvedRoadLines(
      Obb(rangeLength / 2.0, rangeWidth / 2.0, vehicle->getPosition()));

    static bool first = true;
    static Observation observation;
    if (first) {
      std::vector<std::vector<double>> blank(viewLength,
          std::vector<double>(viewWidth, 0.0));
      observation.fill(blank);
      first = false;
    }
    for (auto & a : observation) {
      for (auto & b : a)
        std::fill(b.begin(), b.end(), 0);
    }

    double minX = center.getX() - rangeLength / 2.0,
           minY = center.getY() - rangeWidth / 2.0;

    for (const auto& v : adjacentVehicles) {
      int x = (int)((v->getPosition().getX() - minX) / Magnification),
          y = (int)((v->getPosition().getY() - minY) / Magnification);
      assert(x >= 0 && x < viewLength && y >= 0 && y < viewWidth);
      observation[0][x][y] = 1.0;
      observation[1][x][y] = v->getVelocity() / control::MaxVelocity;
      observation[2][x][y] = wrapToPi(v->getRotation()) / 2 / Pi;
    }

    for (const auto& line : involvedRoadLines) {
      const auto &segment = line.first;
      bool continuous = line.second;
      if (std::abs(segment.getStart().getX() - segment.getEnd().getX()) < eps) {
        int x = (int)((segment.getStart().getX() - minX) / Magnification);
        assert(x >= 0 && x < viewLength);
        for (std::size_t y = 0; y < viewWidth; ++y) {
          if (!continuous && y % 4 >= 2)
            continue;
          observation[0][x][y] = 1.0;
        }
      } else {
        double slope = (segment.getEnd().getY() - segment.getStart().getY()) /
            (segment.getEnd().getX() - segment.getStart().getX());
        for (std::size_t x = 0; x < viewLength; ++x) {
          int y = (int)((slope * (minX + x * Magnification - segment.getStart().getX()) +
              segment.getStart().getY() - minY) / Magnification);
          if (y < 0 || y >= viewWidth)
            continue;
          observation[0][x][y] = 1.0;
        }
      }
    }

    return observation;
}

std::list<std::shared_ptr<control::VehicleController>>
Environment::getAdjacentVehicles(const VectorD &center, double rangeLength,
                                 double rangeWidth) {
//  std::cout << clock() << " " << "start" << std::endl;
  auto gridPos = getGridPos(center);
  std::list<std::shared_ptr<control::VehicleController>> adjacentVehicles;

  int x = gridPos.first, y = gridPos.second;
  int nx = (int)(rangeLength / 2 / Environment::cellX) + 2,
      ny = (int)(rangeWidth / 2 / Environment::cellY) + 2;
  for (int dx = -nx; dx <= nx; ++dx) {
    for (int dy = -ny; dy <= ny; ++dy) {
      std::pair<int, int> gridCoordinate(x + dx, y + dy);
      if (vehicleGrid.find(gridCoordinate) == vehicleGrid.end())
        continue;
      for (auto id : vehicleGrid[std::make_pair(x + dx, y + dy)]) {
        const auto &vehicle = vehicles.at(id);
        auto vehiclePosition = vehicle->getPosition();
        if (std::abs(vehiclePosition.getX() - center.getX()) <
                rangeLength / 2 &&
            std::abs(vehiclePosition.getY() - center.getY()) < rangeWidth / 2) {
          adjacentVehicles.emplace_back(vehicles.at(id));
        }
      }
    }
  }
  adjacentVehicles.sort([&](std::shared_ptr<control::VehicleController> l,
                            std::shared_ptr<control::VehicleController> r) {
    return (l->getPosition() - center).getLength() <
           (r->getPosition() - center).getLength();
  });
//  std::cout << clock() << " " << "end" << std::endl;
  return adjacentVehicles;
}

std::list<std::pair<Segment, bool>>
Environment::getInvolvedRoadLines(const Obb &window) {
  std::list<std::pair<Segment, bool>> involvedLines;
  const auto &roads = roadMap->getRoads();
  for (auto &kv : roads) {
    std::shared_ptr<roadmap::StraightRoad> pSR =
        std::dynamic_pointer_cast<roadmap::StraightRoad>(kv.second);
    if (!pSR)
      continue;
    std::size_t midLine = 0;
    const auto &lanes = pSR->getLanes();
    for (std::size_t i = 1; i < lanes.size(); ++i) {
      if (lanes[i - 1].isReversed() == lanes[i].isReversed())
        continue;
      midLine = i;
      break;
    }
    const auto &edges = pSR->getEdges();
    for (std::size_t i = 0; i < edges.size(); ++i) {
      if (!ds::checkCollision(window, edges[i]))
        continue;
      bool continuous = (i == 0 || i == edges.size() - 1 || i == midLine);
      involvedLines.emplace_back(edges[i], continuous);
    }
  }
  return involvedLines;
}

bool Environment::isIntersect(const std::pair<int, int> &gridPos,
                              const Segment &segment) {
  int x = gridPos.first, y = gridPos.second;
  auto lb = VectorD(x * cellX, y * cellY),
       rb = VectorD((x + 1) * cellX, y * cellY);
  if (ds::isIntersect(Segment(lb, rb), segment))
    return true;
  auto lt = VectorD(x * cellX, (y + 1) * cellY);
  if (ds::isIntersect(Segment(lb, lt), segment))
    return true;
  auto rt = VectorD((x + 1) * cellX, (y + 1) * cellY);
  if (ds::isIntersect(Segment(lt, rt), segment))
    return true;
  return !ds::isIntersect(Segment(rb, rt), segment);
}

bool Environment::isCover(const std::pair<int, int> &gridPos,
                          const VectorD &position) {
  if (position.getX() < gridPos.first * cellX - eps ||
      position.getX() > (gridPos.first + 1) * cellX + eps)
    return false;
  return !(position.getY() < gridPos.second * cellY - eps ||
           position.getY() > (gridPos.second + 1) * cellY + eps);
}

Environment Environment::makeHighway() {
  roadmap::RoadMapBuilder builder = roadmap::RoadMapBuilder();
  // TODO(yifan): how to give out a path?
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind('/'));
  std::cout << dir_path + "/../../test/RoadMap/highway.json" << std::endl;
  builder.parseJSON(dir_path + "/../../test/RoadMap/highway.json");
  builder.buildRoadMap();

  Environment env(builder.getRoadMap(), builder.getGraph());
  env.addVehicle<control::IDMVehicle>(15, 8, 400, 180, 0, 10, 60);
  env.addVehicle<control::AgentVehicle>(15, 8, 200, 180, 0, 10, 80);

  return env;
}

void Environment::loadVehicles(std::list<std::unordered_map<std::string, float>> &vehicles_config) {

  int n_agent = 0, n_social = 0;

  std::cout << "[DEBUG] Environment::loadVehicles n_agent: " << vehicles_config.size() << std::endl;

  static std::mt19937_64 eng(std::random_device{}());
  std::shuffle(generators.begin(), generators.end(), eng);

  auto count = std::min(generators.size(), vehicles_config.size());
      // double safeDis = std::max(config.at("halfWidth"), config.at("halfLength")) * 5.0;
  int i = 0;
  for(auto& config: vehicles_config) {
      const auto generatorPoint = generators.begin() + i;

      const auto &position = generatorPoint->first;
      double rotation = generatorPoint->second;

      config["rotation"] = rotation;
      config["x"] = position.getX();
      config["y"] = position.getY();

      if (int(config.at("type")) == control::VehicleType::Agent) {
          VehicleId id = addVehicle<control::AgentVehicle>(config);

          switch (agentActionType) {
              case control::ACTION_SPACE_TYPE::DISCRETE:
                  action_space.emplace(id, control::discrete::Action::Count);
                  break;
              case control::ACTION_SPACE_TYPE::CONTINUOUS:
              case control::ACTION_SPACE_TYPE::BUCKET:
              default:
                  break;
          }

          n_agent++;
      } else if (int(config.at("type")) == control::VehicleType::Social) {
          VehicleId id = addVehicle<control::IDMVehicle>(config);
          if (id == ULONG_MAX) continue;
          n_social++;
      }

      if (++i >= count) break;
  }

  std::cout << "[DEBUG] Environment::loadVehicles real n_agent: " << n_agent << std::endl;
}

EventContainer Environment::evaluate(VehicleId vehicleId) const {
  std::unordered_map<EventFlag::Type, std::function<Json::Value()>> funcs = {
      std::make_pair(EventFlag::Collision,
                     [vehicleId, this]() -> Json::Value {
                       Json::Value res;
                       res["value"] = vehicles.at(vehicleId)->isCrashed();
                       return res;
                     }),
      std::make_pair(EventFlag::MaxSpeed,
              [vehicleId, this]() -> Json::Value {
          Json::Value res;
          const auto& vehicle = vehicles.at(vehicleId);
          // TODO(ming): fit to road speed limit
          res["value"] = vehicle->getVelocity() / control::MaxVelocity;
          return res;
      }),
      std::make_pair(EventFlag::IllegalDirection,
              [vehicleId, this]() -> Json::Value {
          Json::Value res;
          const auto& vehicle = vehicles.at(vehicleId);

          // get current lane direction
          auto curLane = vehicle->getLaneId();
          if (roadMap->getRoads().at(curLane.first)->getRoadType() == roadmap::RoadType::Straight) {
          	const auto& road = roadMap->getRoad<roadmap::StraightRoad>(curLane.first);
          	const auto& lane = road->getLanes().at(curLane.second);
          	
          	VectorD laneDir;
          	Segment axis = road->getLaneAxis(curLane.second);
          	if (lane.isReversed()) {
          		laneDir = axis.getStart() - axis.getEnd();
          	} else laneDir = axis.getEnd() - axis.getStart();
          	
          	VectorD myDir = VectorD(std::cos(vehicle->getRotation()), std::sin(vehicle->getRotation()));
          	
          	res["value"] = dot(myDir, laneDir) <= 0.0;
          } else res["value"] = false;
          return res;
      }),
      std::make_pair(EventFlag::Follow,
              [vehicleId, this]() -> Json::Value {
          Json::Value res;
          const auto& vehicle = vehicles.at(vehicleId);
          auto curLane = vehicle->getLaneId();
          if (roadMap->getRoads().at(curLane.first)->getRoadType() == roadmap::RoadType::Straight) {
	          auto road = roadMap->getRoad<roadmap::StraightRoad>(curLane.first);
	          const auto &lane = road->getLanes().at(curLane.second);

	          VectorD laneDir;
	          Segment axis = road->getLaneAxis(curLane.second);
	          if (lane.isReversed()) {
		          laneDir = axis.getStart() - axis.getEnd();
	          } else laneDir = axis.getEnd() - axis.getStart();

	          VectorD myDir = VectorD(std::cos(vehicle->getRotation()), std::sin(vehicle->getRotation()));
	          
	          res["value"] = cos(myDir, laneDir);
          } else res["value"] = 1.;
          
          return res;
      }),
      std::make_pair(EventFlag::SwitchLane,
      		[vehicleId, this]() -> Json::Value {
      	Json::Value res;
      	const auto& vehicle = vehicles.at(vehicleId);
      	auto lastLane = vehicle->getLastLaneId();
      	auto curLane = vehicle->getLaneId();

      	if (lastLane.first == curLane.first) {
      		res["value"] = (lastLane.second != curLane.second);
      	} else res["value"] = false;

      	return res;
      }),
      std::make_pair(EventFlag::OverSolidLine,
              [vehicleId, this]() -> Json::Value {
          Json::Value res;
          const auto& vehicle = vehicles.at(vehicleId);
          
          // TODO(ming): check whether oversolidline
          res["value"] = false;
          return res;
      })
  };

  auto v = vehicles.at(vehicleId);
  auto eventMaskVector = v->getEventListening();
  EventContainer res;

  for (auto &kv : funcs) {
    if (eventMaskVector.find(kv.first) == eventMaskVector.end())
      continue;
    res.addEvent(kv.first, kv.second());
  }

  return res;
}

bool Environment::checkOutMap(ds::VehicleId vehicleId) const {
  const auto &v = vehicles.at(vehicleId);
  return v->getX() < anchor1.getX() || v->getY() < anchor1.getY() ||
            v->getX() > anchor2.getX() || v->getY() > anchor2.getY();
}

bool Environment::checkCollision(VehicleId vehicleId) const {
  const auto &v = vehicles.at(vehicleId);
  for (const auto &kv : vehicles) {
    if (kv.first == vehicleId)
      continue;
    if (ds::checkCollision(*kv.second, *v)) {
      return true;
    }
  }

  for (const auto &kv : roadMap->getRoads()) {
    auto road = std::dynamic_pointer_cast<roadmap::StraightRoad>(kv.second);
    if (!road)
      continue;
    auto &edges = road->getEdges();
    if (ds::checkCollision(edges.front(), *v))
      return true;
    if (ds::checkCollision(edges.back(), *v))
      return true;
  }

  return false;
}

void Environment::setAnchor1(const VectorD &anchor) {
  Environment::anchor1 = anchor;
}

void Environment::setAnchor2(const VectorD &anchor) {
  Environment::anchor2 = anchor;
}

void Environment::updateAnchor1(const VectorD &anchor) {
  Environment::anchor1.getX() =
      std::min(Environment::anchor1.getX(), anchor.getX());
  Environment::anchor1.getY() =
      std::min(Environment::anchor1.getY(), anchor.getY());
}

void Environment::updateAnchor2(const VectorD &anchor) {
  Environment::anchor2.getX() =
      std::max(Environment::anchor2.getX(), anchor.getX());
  Environment::anchor2.getY() =
      std::max(Environment::anchor2.getY(), anchor.getY());
}

  void Environment::setMaxVehicle(size_t maxVehicle) {
    Environment::maxVehicle = maxVehicle;
  }

} // namespace engine
} // namespace ds

namespace ds {
namespace engine {

void printObservation(const Observation &observation) {
  assert(!observation[0].empty());
  std::size_t length = observation[0].size(), width = observation[0][0].size();

  for (std::size_t i = 0; i < length; ++i) {
    for (std::size_t j = 0; j < width; ++j) {
      if (std::abs(observation[0][i][j]) < eps)
        std::cout << ' ';
      else
        std::cout << 1;
    }
    std::cout << std::endl;
  }
}

} // namespace engine
} // namespace ds