set(HEADERS_PATH ${CMAKE_SOURCE_DIR}/include/DriveSingularity/Render)

set(HEADERS
    ${HEADERS_PATH}/EnvironmentRenderer.h
    ${HEADERS_PATH}/RoadMapRenderer.h
    )

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/EnvironmentRenderer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/RoadMapRenderer.cpp
    )

add_library(${PROJECT_NAME}-renderer ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME}-renderer
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    PRIVATE
    ${CMAKE_SOURCE_DIR}/include/DriveSingularity
    )

target_compile_features(${PROJECT_NAME}-renderer PRIVATE cxx_std_14)
if (UNIX)
  target_compile_options(${PROJECT_NAME}-renderer PRIVATE -o2 -Wall)
endif ()

find_package(SDL2 REQUIRED)
target_include_directories(${PROJECT_NAME}-renderer
    PUBLIC
    ${SDL2_INCLUDE_DIRS}
    )
target_link_libraries(${PROJECT_NAME}-renderer
    PUBLIC
    ${SDL2_LIBRARIES} ${PROJECT_NAME}
    )
