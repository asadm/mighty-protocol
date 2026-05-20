cmake_minimum_required(VERSION 3.16)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES AND EXISTS "/opt/homebrew")
  set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "macOS target architecture" FORCE)
endif()

project(mighty_mapper_cpp_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(MIGHTY_PROTOCOL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
set(MIGHTY_MAPPER_ROOT "${MIGHTY_PROTOCOL_ROOT}/mighty-algorithms/mighty-mapper")

list(PREPEND CMAKE_MODULE_PATH "${MIGHTY_MAPPER_ROOT}/cmake")
list(PREPEND CMAKE_PREFIX_PATH "${REPO_ROOT}/external/pangolin-install")
list(PREPEND CMAKE_PREFIX_PATH "/opt/homebrew/opt/eigen")

if(APPLE AND CMAKE_OSX_ARCHITECTURES MATCHES "arm64" AND EXISTS "/opt/homebrew/lib/libcholmod.dylib")
  set(CHOLMOD_INCLUDE_DIR "/opt/homebrew/include/suitesparse" CACHE PATH "CHOLMOD include path" FORCE)
  set(CSPARSE_INCLUDE_DIR "/opt/homebrew/include/suitesparse" CACHE PATH "CXSparse include path" FORCE)
  set(CHOLMOD_LIBRARY "/opt/homebrew/lib/libcholmod.dylib" CACHE FILEPATH "CHOLMOD library" FORCE)
  set(CSPARSE_LIBRARY "/opt/homebrew/lib/libcxsparse.dylib" CACHE FILEPATH "CXSparse library" FORCE)
  set(AMD_LIBRARY "/opt/homebrew/lib/libamd.dylib" CACHE FILEPATH "AMD library" FORCE)
  set(CAMD_LIBRARY "/opt/homebrew/lib/libcamd.dylib" CACHE FILEPATH "CAMD library" FORCE)
  set(CCOLAMD_LIBRARY "/opt/homebrew/lib/libccolamd.dylib" CACHE FILEPATH "CCOLAMD library" FORCE)
  set(COLAMD_LIBRARY "/opt/homebrew/lib/libcolamd.dylib" CACHE FILEPATH "COLAMD library" FORCE)
  set(METIS_LIBRARY "/opt/homebrew/lib/libmetis.dylib" CACHE FILEPATH "METIS library" FORCE)
  set(SUITESPARSECONFIG_LIBRARY "/opt/homebrew/lib/libsuitesparseconfig.dylib" CACHE FILEPATH "SuiteSparse config library" FORCE)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui)
find_package(Eigen3 REQUIRED)
find_package(SuiteParse REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(LibZip QUIET)

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

if(NOT TARGET mighty_mapper_core)
  add_subdirectory("${MIGHTY_MAPPER_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/mighty-mapper" EXCLUDE_FROM_ALL)
endif()

add_executable(mighty_mapper_live main.cpp)

target_include_directories(mighty_mapper_live PRIVATE
  "${MIGHTY_PROTOCOL_ROOT}/cpp"
  "${MIGHTY_MAPPER_ROOT}/include"
  "${MIGHTY_MAPPER_ROOT}/src"
  "${MIGHTY_MAPPER_ROOT}/thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR}
)

target_link_libraries(mighty_mapper_live PRIVATE
  mighty_mapper_core
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBZIP_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
)
