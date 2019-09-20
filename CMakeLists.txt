project(drive-singularity)

cmake_minimum_required(VERSION 3.8)

option(ENABLE_TESTING "" OFF)

find_package(PythonInterp 3 REQUIRED)
set(PYBIND11_PYTHON_VERSION 3)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(pybind11)
add_subdirectory(src)
if (ENABLE_TESTING)
  add_subdirectory(gtest)
  add_subdirectory(test)
endif()

pybind11_add_module(simulator src/simulator.cpp)
target_link_libraries(simulator PRIVATE ${PROJECT_NAME})

add_custom_command(TARGET simulator POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:simulator>
  ${CMAKE_SOURCE_DIR}/pyds
  )
