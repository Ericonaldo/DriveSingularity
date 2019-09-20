#include "DriveSingularity/Engine/Environment.h"
#include "DriveSingularity/Utils/Events.h"
#include <DriveSingularity/Control/AgentVehicle.h>
#include <DriveSingularity/RoadMap/RoadMapBuilder.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using ds::control::AgentVehicle;
using ds::engine::Environment;
using ds::engine::FeedBack;
using ds::roadmap::Graph;
using ds::roadmap::RoadMap;
using ds::roadmap::RoadMapBuilder;

PYBIND11_MODULE(simulator, m) {
  using namespace ds;

  py::class_<EventFlag>(m, "EventFlag")
      .def_readonly_static("None", &EventFlag::None)
      .def_readonly_static("Follow", &EventFlag::Follow)
      .def_readonly_static("Collision", &EventFlag::Collision)
      .def_readonly_static("OverSolidLine", &EventFlag::OverSolidLine)
      .def_readonly_static("IllegalDirection", &EventFlag::IllegalDirection);

  py::class_<EventContainer>(m, "EventContainer")
      .def("get_event",
           [](const EventContainer &events, EventFlag::Type event) {
             Json::StreamWriterBuilder builder;
             std::string output = Json::writeString(builder, events[event]);
             return output;
           });

  py::class_<FeedBack>(m, "FeedBack")
      .def(py::init<std::size_t, engine::Observation, double, bool>())
      .def_readwrite("id", &FeedBack::id)
      .def_readwrite("observation", &FeedBack::observation)
      .def_readwrite("done", &FeedBack::done)
      .def_readwrite("events", &FeedBack::events);

  // ************* Environment ****************** //
  py::class_<Environment>(m, "Environment")
      .def_readonly("action_space", &Environment::action_space)
      .def(py::init<RoadMap, Graph>())
      .def("load_vehicles", &Environment::loadVehicles,  // TODO(ming): reinfe load vehicles
           "A function for generate vehicles with given configuration.",
           py::arg("vehicles_config")) // load vehicles, also agents with given
                                       // configuration.
      .def("load_generators", &Environment::loadGenerators,
           "A function for creating social vehicle generators with given "
           "configuration.",
           py::arg("generator_config"))
      .def("set_social_vehicle_generator", &Environment::initGenerator)
      .def("agents", &Environment::getAgents,
           "Retrieve all alive agents.") // agent ids.
      .def("get_observation", &Environment::getObservation,
           "Retrieve agent's observation with given agent id, and visible "
           "radius.",
           py::arg("agent_id"), py::arg("view_length") = 84,
           py::arg("view_width") = 84) // observation: given radius.
      .def("add_listener", &Environment::addListener,
           "Register event listener for reward calculation.",
           py::arg("agent_id"), py::arg("event_type"))
      .def("set_render", &Environment::setLogger, "Initialize logger",
           py::arg("f_path"))
      .def("turn_on_render", &Environment::turnOnRecord)
      .def("turn_off_render", &Environment::turnOffRecord)
      .def("save_render", &Environment::saveRecord)
      .def("reset", &Environment::reset)
      .def("step",
           (std::vector<FeedBack>(Environment::*)(
               const std::unordered_map<std::size_t, std::size_t> &action)) &
               Environment::step,
           py::arg("actions"));

  // ************* Agent ***************** //
  py::class_<AgentVehicle, std::shared_ptr<AgentVehicle>>(
      m, "Agent", py::multiple_inheritance())
      .def(py::init<const std::shared_ptr<RoadMap> &,
                    const std::shared_ptr<Graph> &, double, double, double,
                    double, double, double, double, ds::roadmap::LaneId,
                    ds::roadmap::LaneId>())
      .def("alive", &AgentVehicle::isCrashed);

  // ********** RoadMap ************ //
  py::class_<RoadMap>(m, "RoadMap").def(py::init<>());

  // ********** RoadGraph ************ //
  py::class_<Graph>(m, "Graph").def(py::init());

  // ********** RoadMapBuilder ********* //
  py::class_<RoadMapBuilder>(m, "RoadMapBuilder")
      .def(py::init<>())
      .def("load", &RoadMapBuilder::parseJSON, py::arg("file_path"))
      .def("build", &RoadMapBuilder::buildRoadMap)
      .def("get_road_map", &RoadMapBuilder::getRoadMap)
      .def("get_road_graph", &RoadMapBuilder::getGraph);
}
