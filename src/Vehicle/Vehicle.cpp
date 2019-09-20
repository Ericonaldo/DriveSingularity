#include "Vehicle/Vehicle.h"
#include <json/value.h>

namespace ds {
Json::Value VehicleEntity::retrieveInfo(bool flag, size_t timestep) {
  // TODO(ming): return a json block
  // position, width, height, rotation
  Json::Value obj, vec(Json::arrayValue);

  vec.append(Json::Value(getPosition().getX()));
  vec.append(Json::Value(getPosition().getY()));

  obj["position"] = vec;
  obj["rotation"] = getRotation();
  obj["timestep"] = Json::UInt64(timestep);
  
  if (flag) {
	  obj["width"] = getHalfWidth() * 2.0;
	  obj["height"] = getHalfLength() * 2.0;
  }

  return obj;
}
} // namespace ds