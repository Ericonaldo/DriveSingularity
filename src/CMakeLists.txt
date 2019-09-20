set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(HEADERS_PATH ${CMAKE_SOURCE_DIR}/include/DriveSingularity)

set(HEADERS
  ${HEADERS_PATH}/Control/simple/IDMVehicle.h
  ${HEADERS_PATH}/Control/AgentVehicle.h
  ${HEADERS_PATH}/Control/Control.h
  ${HEADERS_PATH}/Control/Action.h

  ${HEADERS_PATH}/Engine/Environment.h
  ${HEADERS_PATH}/Engine/Route.h
#  ${HEADERS_PATH}/Engine/Scenario.h

  ${HEADERS_PATH}/Math/All.h
  ${HEADERS_PATH}/Math/Collision.h
  ${HEADERS_PATH}/Math/Constants.h
  ${HEADERS_PATH}/Math/Interval.h
  ${HEADERS_PATH}/Math/Obb.h
  ${HEADERS_PATH}/Math/Random.h
  ${HEADERS_PATH}/Math/Segment.h
  ${HEADERS_PATH}/Math/Util.h
  ${HEADERS_PATH}/Math/Vector.h

  ${HEADERS_PATH}/RoadMap/Common.h
  ${HEADERS_PATH}/RoadMap/Graph.h
  ${HEADERS_PATH}/RoadMap/RoadMap.h
  ${HEADERS_PATH}/RoadMap/RoadMapBuilder.h
  ${HEADERS_PATH}/RoadMap/Roads.h

  ${HEADERS_PATH}/Utils/Creator.h
  ${HEADERS_PATH}/Utils/Logger.h
  ${HEADERS_PATH}/Utils/VehicleGenerator.h
  ${HEADERS_PATH}/Utils/Events.h

  ${HEADERS_PATH}/Vehicle/Common.h
  ${HEADERS_PATH}/Vehicle/Vehicle.h
  )

set(SOURCES
  Control/simple/IDMVehicle.cpp
  Control/AgentVehicle.cpp
  Control/Control.cpp

  Engine/Environment.cpp
    Engine/Route.cpp

  Math/Collision.cpp
  Math/Obb.cpp

  RoadMap/Graph.cpp
  RoadMap/RoadMap.cpp
  RoadMap/RoadMapBuilder.cpp
  RoadMap/Roads.cpp

  Utils/Events.cpp
  Utils/Logger.cpp
  Utils/VehicleGenerator.cpp

  Vehicle/Vehicle.cpp

  ${CMAKE_SOURCE_DIR}/pugixml/pugixml.cpp
  )

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_include_directories(${PROJECT_NAME}
  PUBLIC ${CMAKE_SOURCE_DIR}/include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include/DriveSingularity
    ${CMAKE_SOURCE_DIR}/pugixml/
  )
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
if (UNIX)
  target_compile_options(${PROJECT_NAME} PRIVATE -o2 -Wall)
endif ()

find_package(jsoncpp REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC jsoncpp_lib)

# renderer
#if (ENABLE_TESTING)
  add_subdirectory(Render)
#endif()