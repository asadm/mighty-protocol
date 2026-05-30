cmake_minimum_required(VERSION 3.16)

project(mighty_cpp_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

if(DEFINED ENV{MIGHTY_ALGORITHMS_SDK} AND NOT MightyAlgorithms_DIR)
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{MIGHTY_ALGORITHMS_SDK}")
endif()
if(NOT MightyAlgorithms_DIR)
  set(_MIGHTY_ALGORITHMS_BUNDLED_SDK
    "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/algorithms/current")
  if(EXISTS "${_MIGHTY_ALGORITHMS_BUNDLED_SDK}/lib/cmake/MightyAlgorithms/MightyAlgorithmsConfig.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${_MIGHTY_ALGORITHMS_BUNDLED_SDK}")
  endif()
endif()
find_package(MightyAlgorithms QUIET)

add_executable(mightyapp main.cpp)

target_include_directories(mightyapp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp
)

target_link_libraries(mightyapp PRIVATE ${OpenCV_LIBS})

if(MightyAlgorithms_FOUND)
  add_executable(mighty_mapper_sdk_example mapper_sdk/main.cpp)
  target_link_libraries(mighty_mapper_sdk_example PRIVATE
    MightyAlgorithms::mighty_algorithms)

  add_executable(mighty_loopclosure_sdk_example loopclosure_sdk/main.cpp)
  target_link_libraries(mighty_loopclosure_sdk_example PRIVATE
    MightyAlgorithms::mighty_algorithms)
else()
  message(STATUS "MightyAlgorithms package not found; SDK examples disabled.")
endif()
