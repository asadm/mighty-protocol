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

list(PREPEND CMAKE_PREFIX_PATH "${REPO_ROOT}/external/pangolin-install")
list(PREPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/eigen")

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(GLEW REQUIRED)

if(DEFINED ENV{MIGHTY_LOOPCLOSURE_SDK})
  set(_MIGHTY_LOOPCLOSURE_SDK "$ENV{MIGHTY_LOOPCLOSURE_SDK}")
else()
  set(_MIGHTY_LOOPCLOSURE_SDK
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../lib/loopclosure/macos-arm64-static")
endif()
set(_MIGHTY_LOOPCLOSURE_INCLUDE_DIR "${_MIGHTY_LOOPCLOSURE_SDK}/include")
if(APPLE)
  set(_MIGHTY_LOOPCLOSURE_LIBRARY
    "${_MIGHTY_LOOPCLOSURE_SDK}/lib/libmighty_loopclosure_device.dylib")
elseif(WIN32)
  set(_MIGHTY_LOOPCLOSURE_LIBRARY
    "${_MIGHTY_LOOPCLOSURE_SDK}/lib/mighty_loopclosure_device.dll")
else()
  set(_MIGHTY_LOOPCLOSURE_LIBRARY
    "${_MIGHTY_LOOPCLOSURE_SDK}/lib/libmighty_loopclosure_device.so")
endif()
if(NOT EXISTS "${_MIGHTY_LOOPCLOSURE_LIBRARY}")
  message(FATAL_ERROR "mighty loopclosure SDK library not found: ${_MIGHTY_LOOPCLOSURE_LIBRARY}")
endif()

add_library(MightyLoopClosureDevice::mighty_loopclosure_device SHARED IMPORTED)
set_target_properties(MightyLoopClosureDevice::mighty_loopclosure_device PROPERTIES
  IMPORTED_LOCATION "${_MIGHTY_LOOPCLOSURE_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${_MIGHTY_LOOPCLOSURE_INCLUDE_DIR}")

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

get_filename_component(_MIGHTY_LOOPCLOSURE_LIB_DIR
  "${_MIGHTY_LOOPCLOSURE_LIBRARY}" DIRECTORY)
if(APPLE AND _MIGHTY_LOOPCLOSURE_LIB_DIR)
  target_link_options(mighty_mapper_live PRIVATE
    "LINKER:-rpath,${_MIGHTY_LOOPCLOSURE_LIB_DIR}")
endif()

target_include_directories(mighty_mapper_live PRIVATE
  "${MIGHTY_PROTOCOL_ROOT}/cpp"
  "${_MIGHTY_LOOPCLOSURE_INCLUDE_DIR}"
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
)

target_link_libraries(mighty_mapper_live PRIVATE
  MightyLoopClosureDevice::mighty_loopclosure_device
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  Threads::Threads
  GLEW::GLEW
)
