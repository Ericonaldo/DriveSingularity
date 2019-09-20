//
// Created by ming on 8/6/19.
// Logger is not thread-safe
//

#ifndef DRIVE_SINGULARITY_LOGGER_H
#define DRIVE_SINGULARITY_LOGGER_H

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <json/json.h>
#include "DriveSingularity/Control/Control.h"

namespace ds {
namespace utils {

class Logger {
public:
  explicit Logger() { fpath = ""; }
  ~Logger();

  void log(size_t timestep, std::unordered_map<VehicleId, std::shared_ptr<control::VehicleController>>&);
  void log(const std::unordered_map<roadmap::NodeId, std::shared_ptr<roadmap::Road>>&);
  void writeToFile();
  void clear() { vsLog.clear(); nodeLog.clear(); }
  void setFilePath(std::string& fPath) {
  	fpath.assign(fPath);
  }

private:
	std::string fpath;
  std::unordered_map<VehicleId, Json::Value> vsLog;
  std::unordered_map<roadmap::NodeId, Json::Value> nodeLog;
};
} // namespace utils
} // namespace ds
#endif // DRIVE_SINGULARITY_LOGGER_H
