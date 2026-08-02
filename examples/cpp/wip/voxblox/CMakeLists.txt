cmake_minimum_required(VERSION 3.16)

project(mighty_voxblox_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
include(cmake/voxblox_standalone.cmake)

get_filename_component(MIGHTY_REPOSITORY_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../.." ABSOLUTE)
set(MIGHTY_PANGOLIN_ROOT "" CACHE PATH
  "Optional Pangolin installation prefix")
if(MIGHTY_PANGOLIN_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${MIGHTY_PANGOLIN_ROOT}")
elseif(EXISTS "${MIGHTY_REPOSITORY_ROOT}/external/pangolin-install")
  list(PREPEND CMAKE_PREFIX_PATH
    "${MIGHTY_REPOSITORY_ROOT}/external/pangolin-install")
endif()

find_package(Pangolin 0.2 QUIET)
if(NOT Pangolin_FOUND)
  message(FATAL_ERROR
    "Pangolin is required for the native voxblox viewer. Install Pangolin "
    "or configure with -DMIGHTY_PANGOLIN_ROOT=/path/to/pangolin/install")
endif()

if(TARGET pango_display)
  set(_mighty_pangolin_target pango_display)
elseif(TARGET Pangolin::pangolin)
  set(_mighty_pangolin_target Pangolin::pangolin)
else()
  set(_mighty_pangolin_target ${Pangolin_LIBRARIES})
endif()

add_executable(mighty_voxblox
  main.cpp
  voxblox_mapper.cpp
  voxblox_viewer.cpp
)

target_include_directories(mighty_voxblox PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../cpp
  ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(mighty_voxblox PRIVATE
  mighty_voxblox_core
  ${_mighty_pangolin_target}
  Threads::Threads
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(mighty_voxblox PRIVATE -Wall -Wextra -Wpedantic)
endif()
