cmake_minimum_required(VERSION 3.16)

project(mighty_voxblox_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Upstream voxblox exports a catkin package config. Source the workspace that
# contains voxblox before configuring this standalone example.
find_package(voxblox REQUIRED)
find_package(Threads REQUIRED)

add_executable(mighty_voxblox
  main.cpp
  voxblox_mapper.cpp
)

target_include_directories(mighty_voxblox PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp
  ${voxblox_INCLUDE_DIRS}
)

if(TARGET voxblox::voxblox)
  target_link_libraries(mighty_voxblox PRIVATE
    voxblox::voxblox
    Threads::Threads
  )
elseif(TARGET voxblox)
  target_link_libraries(mighty_voxblox PRIVATE
    voxblox
    Threads::Threads
  )
else()
  target_link_libraries(mighty_voxblox PRIVATE
    ${voxblox_LIBRARIES}
    Threads::Threads
  )
endif()

if(voxblox_EXPORTED_TARGETS)
  add_dependencies(mighty_voxblox ${voxblox_EXPORTED_TARGETS})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(mighty_voxblox PRIVATE -Wall -Wextra -Wpedantic)
endif()
