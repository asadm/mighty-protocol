cmake_minimum_required(VERSION 3.16)

project(mighty_occupancy_grid_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

get_filename_component(MIGHTY_PROTOCOL_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
if(NOT MightyAlgorithms_DIR)
  set(MightyAlgorithms_DIR
    "${MIGHTY_PROTOCOL_ROOT}/lib/algorithms/current/lib/cmake/MightyAlgorithms")
endif()
find_package(MightyAlgorithms CONFIG REQUIRED)

add_executable(mighty_occupancy_grid_example main.cpp)
target_compile_definitions(mighty_occupancy_grid_example PRIVATE
  MIGHTY_PROTOCOL_ENABLE_OCCUPANCY_GRID=1)
target_include_directories(mighty_occupancy_grid_example PRIVATE
  "${MIGHTY_PROTOCOL_ROOT}")
target_link_libraries(mighty_occupancy_grid_example PRIVATE
  MightyAlgorithms::mighty_algorithms
  Threads::Threads)
