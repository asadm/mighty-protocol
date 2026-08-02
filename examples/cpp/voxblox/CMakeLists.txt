cmake_minimum_required(VERSION 3.16)

project(mighty_voxblox_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
include(cmake/voxblox_standalone.cmake)

add_executable(mighty_voxblox
  main.cpp
  voxblox_mapper.cpp
)

target_include_directories(mighty_voxblox PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp
)

target_link_libraries(mighty_voxblox PRIVATE
  mighty_voxblox_core
  Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(mighty_voxblox PRIVATE -Wall -Wextra -Wpedantic)
endif()
