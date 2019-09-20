//
// Created by ming on 8/6/19.
//

#include <DriveSingularity/Vehicle/Common.h>
#include "DriveSingularity/Utils/Logger.h"

namespace ds {
namespace utils {

Logger::~Logger() = default;

void Logger::log(size_t timestep, std::unordered_map<VehicleId, std::shared_ptr<control::VehicleController>> & vehicles) {
	for (auto& kv: vehicles) {
		auto key = kv.first;
		const auto& vehicle = kv.second;

		bool create = false;
		if (vsLog.find(key) == vsLog.end()) {
			vsLog.emplace(key, Json::Value(Json::arrayValue));
			create = true;
		}

		auto message = vehicle->retrieveInfo(create, timestep);

		if (create) message["type"] = vehicle->getVehicleType();
		vsLog.at(key).append(message);
	}
}

void Logger::log(const std::unordered_map<roadmap::NodeId, std::shared_ptr<roadmap::Road>> & roads) {
	for (auto& kv: roads) {
		auto key = kv.first;
		const auto& node = kv.second;

		if (node->getNodeType() == roadmap::NodeType::Crossroads) {
			if (nodeLog.find(key) == nodeLog.end()) nodeLog.emplace(key, Json::Value(Json::arrayValue));
			auto message = node->retrieveInfo();
			nodeLog.at(key).append(message);
		}
	}
}

void Logger::writeToFile() {
	Json::StyledWriter styledWriter;
	Json::Value obj;

	for (auto& kv : vsLog) {
		obj[std::to_string(kv.first)] = kv.second;
	}

	std::shared_ptr<std::ofstream> mLogFile = std::make_shared<std::ofstream>();
	mLogFile->open(fpath + "/render.json");

	*mLogFile << styledWriter.write(obj) << std::endl;
	mLogFile->close();

	mLogFile->open(fpath + "/light.json");
	obj.clear();

	for (auto& kv: nodeLog) obj[std::to_string(kv.first)] = kv.second;

	*mLogFile << styledWriter.write(obj) << std::endl;
	mLogFile->close();
}

} // namespace utils
} // namespace ds
