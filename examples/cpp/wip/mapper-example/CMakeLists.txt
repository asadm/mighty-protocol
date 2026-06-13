cmake_minimum_required(VERSION 3.16)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES AND EXISTS "/opt/homebrew")
  set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "macOS target architecture" FORCE)
endif()

project(mighty_mapper_cpp_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(MIGHTY_PROTOCOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../.." ABSOLUTE)

if(DEFINED ENV{MIGHTY_ALGORITHMS_SDK} AND NOT MightyAlgorithms_DIR)
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{MIGHTY_ALGORITHMS_SDK}")
endif()
if(NOT MightyAlgorithms_DIR)
  set(_MIGHTY_ALGORITHMS_BUNDLED_SDK
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/algorithms/current")
  if(EXISTS "${_MIGHTY_ALGORITHMS_BUNDLED_SDK}/lib/cmake/MightyAlgorithms/MightyAlgorithmsConfig.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${_MIGHTY_ALGORITHMS_BUNDLED_SDK}")
  endif()
endif()

list(PREPEND CMAKE_PREFIX_PATH "${REPO_ROOT}/external/pangolin-install")
list(PREPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/eigen")

find_package(MightyAlgorithms REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)

get_target_property(_MIGHTY_ALGORITHMS_IMPORTED_LOCATION
  MightyAlgorithms::mighty_algorithms IMPORTED_LOCATION_RELEASE)
if(NOT _MIGHTY_ALGORITHMS_IMPORTED_LOCATION)
  get_target_property(_MIGHTY_ALGORITHMS_IMPORTED_LOCATION
    MightyAlgorithms::mighty_algorithms IMPORTED_LOCATION)
endif()
if(_MIGHTY_ALGORITHMS_IMPORTED_LOCATION)
  get_filename_component(_MIGHTY_ALGORITHMS_LIB_DIR
    "${_MIGHTY_ALGORITHMS_IMPORTED_LOCATION}" DIRECTORY)
endif()
if(APPLE AND _MIGHTY_ALGORITHMS_LIB_DIR)
  set(_MIGHTY_PACKAGED_OPENCV_LIBS)
  foreach(_opencv_component calib3d features2d imgproc flann core)
    file(GLOB _opencv_candidates
      "${_MIGHTY_ALGORITHMS_LIB_DIR}/libopencv_${_opencv_component}.*.dylib")
    if(_opencv_candidates)
      list(SORT _opencv_candidates)
      list(GET _opencv_candidates 0 _opencv_candidate)
      list(APPEND _MIGHTY_PACKAGED_OPENCV_LIBS "${_opencv_candidate}")
    endif()
  endforeach()
  if(_MIGHTY_PACKAGED_OPENCV_LIBS)
    set(OpenCV_LIBS ${_MIGHTY_PACKAGED_OPENCV_LIBS})
  endif()
endif()

if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()
if(NOT LIBZIP_LIBRARY)
  set(LIBZIP_LIBRARY "")
endif()

find_package(Pangolin 0.2 QUIET)
if(NOT Pangolin_FOUND AND TARGET pango_display)
  set(Pangolin_FOUND TRUE)
  set(Pangolin_INCLUDE_DIRS "${REPO_ROOT}/external/pangolin-install/include")
  set(Pangolin_LIBRARIES
    pango_core pango_display pango_geometry pango_glgeometry pango_image
    pango_opengl pango_packetstream pango_plot pango_scene pango_tools
    pango_vars pango_video pango_windowing tinyobj)
endif()
if(NOT Pangolin_FOUND)
  message(FATAL_ERROR "Pangolin is required for mapper-example")
endif()

add_executable(mighty_mapper_live main.cpp)

if(APPLE AND _MIGHTY_ALGORITHMS_LIB_DIR)
  # Prefer the packaged SDK dependency set before Homebrew rpaths.
  target_link_options(mighty_mapper_live PRIVATE
    "LINKER:-rpath,${_MIGHTY_ALGORITHMS_LIB_DIR}")
endif()

target_include_directories(mighty_mapper_live PRIVATE
  "${MIGHTY_PROTOCOL_ROOT}/cpp"
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
)

target_link_libraries(mighty_mapper_live PRIVATE
  MightyAlgorithms::mighty_algorithms
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  Threads::Threads
  GLEW::GLEW
)
