include_guard(GLOBAL)

include(FetchContent)

set(MIGHTY_VOXBLOX_SOURCE_DIR "" CACHE PATH
    "Optional local checkout of ethz-asl/voxblox (repository root)")
set(MIGHTY_MINKINDR_SOURCE_DIR "" CACHE PATH
    "Optional local checkout of ethz-asl/minkindr (repository root)")

# Pin the repositories used by the upstream voxblox rosinstall manifest. They
# are fetched only when a local source directory was not supplied.
set(_mighty_voxblox_git_tag c8066b04075d2fee509de295346b1c0b788c4f38)
set(_mighty_minkindr_git_tag 564f12639a8447d4d3e5e7707851424302941056)

if(MIGHTY_VOXBLOX_SOURCE_DIR)
  get_filename_component(_mighty_voxblox_repo
    "${MIGHTY_VOXBLOX_SOURCE_DIR}" ABSOLUTE)
else()
  FetchContent_Declare(mighty_voxblox_source
    GIT_REPOSITORY https://github.com/ethz-asl/voxblox.git
    GIT_TAG ${_mighty_voxblox_git_tag}
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  FetchContent_GetProperties(mighty_voxblox_source)
  if(NOT mighty_voxblox_source_POPULATED)
    FetchContent_Populate(mighty_voxblox_source)
  endif()
  set(_mighty_voxblox_repo "${mighty_voxblox_source_SOURCE_DIR}")
endif()

if(MIGHTY_MINKINDR_SOURCE_DIR)
  get_filename_component(_mighty_minkindr_repo
    "${MIGHTY_MINKINDR_SOURCE_DIR}" ABSOLUTE)
else()
  FetchContent_Declare(mighty_minkindr_source
    GIT_REPOSITORY https://github.com/ethz-asl/minkindr.git
    GIT_TAG ${_mighty_minkindr_git_tag}
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )
  FetchContent_GetProperties(mighty_minkindr_source)
  if(NOT mighty_minkindr_source_POPULATED)
    FetchContent_Populate(mighty_minkindr_source)
  endif()
  set(_mighty_minkindr_repo "${mighty_minkindr_source_SOURCE_DIR}")
endif()

set(_mighty_voxblox_root "${_mighty_voxblox_repo}/voxblox")
set(_mighty_minkindr_include "${_mighty_minkindr_repo}/minkindr/include")
if(NOT EXISTS "${_mighty_voxblox_root}/include/voxblox/core/tsdf_map.h")
  message(FATAL_ERROR
    "MIGHTY_VOXBLOX_SOURCE_DIR does not point to an ethz-asl/voxblox checkout")
endif()
if(NOT EXISTS "${_mighty_minkindr_include}/kindr/minimal/quat-transformation.h")
  message(FATAL_ERROR
    "MIGHTY_MINKINDR_SOURCE_DIR does not point to an ethz-asl/minkindr checkout")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Prefer protobuf's package config when available. Older CMake FindProtobuf
# versions misread protobuf's newer 7.x/34.x version scheme and print a false
# compiler/library mismatch warning. Linux distributions without a config file
# fall back to CMake's module normally.
find_package(Protobuf CONFIG QUIET)
if(NOT Protobuf_FOUND)
  find_package(Protobuf REQUIRED)
endif()
find_package(gflags REQUIRED)
find_package(glog REQUIRED)

if(TARGET protobuf::libprotobuf)
  set(_mighty_protobuf_library protobuf::libprotobuf)
else()
  set(_mighty_protobuf_library ${Protobuf_LIBRARIES})
endif()

if(TARGET protobuf::protoc)
  set(_mighty_protoc $<TARGET_FILE:protobuf::protoc>)
elseif(Protobuf_PROTOC_EXECUTABLE)
  set(_mighty_protoc "${Protobuf_PROTOC_EXECUTABLE}")
else()
  message(FATAL_ERROR "protoc is required to build standalone voxblox")
endif()

if(TARGET glog::glog)
  set(_mighty_glog_library glog::glog)
elseif(TARGET glog)
  set(_mighty_glog_library glog)
else()
  set(_mighty_glog_library ${glog_LIBRARIES})
endif()

if(TARGET gflags::gflags)
  set(_mighty_gflags_library gflags::gflags)
elseif(TARGET gflags)
  set(_mighty_gflags_library gflags)
elseif(TARGET gflags_shared)
  set(_mighty_gflags_library gflags_shared)
else()
  set(_mighty_gflags_library ${gflags_LIBRARIES})
endif()

set(_mighty_voxblox_proto_dir "${_mighty_voxblox_root}/proto")
set(_mighty_voxblox_generated_dir
    "${CMAKE_CURRENT_BINARY_DIR}/voxblox_generated")
set(_mighty_voxblox_proto_files
  "${_mighty_voxblox_proto_dir}/voxblox/Block.proto"
  "${_mighty_voxblox_proto_dir}/voxblox/Layer.proto"
)
set(_mighty_voxblox_generated_sources
  "${_mighty_voxblox_generated_dir}/voxblox/Block.pb.cc"
  "${_mighty_voxblox_generated_dir}/voxblox/Layer.pb.cc"
)
set(_mighty_voxblox_generated_headers
  "${_mighty_voxblox_generated_dir}/voxblox/Block.pb.h"
  "${_mighty_voxblox_generated_dir}/voxblox/Layer.pb.h"
)

file(MAKE_DIRECTORY "${_mighty_voxblox_generated_dir}/voxblox")
add_custom_command(
  OUTPUT
    ${_mighty_voxblox_generated_sources}
    ${_mighty_voxblox_generated_headers}
  COMMAND ${_mighty_protoc}
    --proto_path=${_mighty_voxblox_proto_dir}
    --cpp_out=${_mighty_voxblox_generated_dir}
    ${_mighty_voxblox_proto_files}
  DEPENDS ${_mighty_voxblox_proto_files}
  COMMENT "Generating standalone voxblox protobuf sources"
  VERBATIM
)

# This is the non-ROS subset used by the example: TSDF integration, marching
# cubes, PLY output, and protobuf layer serialization.
add_library(mighty_voxblox_core STATIC
  ${_mighty_voxblox_generated_sources}
  "${_mighty_voxblox_root}/src/core/block.cc"
  "${_mighty_voxblox_root}/src/core/tsdf_map.cc"
  "${_mighty_voxblox_root}/src/integrator/integrator_utils.cc"
  "${_mighty_voxblox_root}/src/integrator/tsdf_integrator.cc"
  "${_mighty_voxblox_root}/src/io/mesh_ply.cc"
  "${_mighty_voxblox_root}/src/mesh/marching_cubes.cc"
  "${_mighty_voxblox_root}/src/utils/protobuf_utils.cc"
  "${_mighty_voxblox_root}/src/utils/timing.cc"
)

target_include_directories(mighty_voxblox_core SYSTEM PUBLIC
  "${_mighty_voxblox_root}/include"
  "${_mighty_voxblox_generated_dir}"
  "${_mighty_minkindr_include}"
  ${Protobuf_INCLUDE_DIRS}
  ${gflags_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
)

target_link_libraries(mighty_voxblox_core PUBLIC
  Eigen3::Eigen
  ${_mighty_protobuf_library}
  ${_mighty_gflags_library}
  ${_mighty_glog_library}
  Threads::Threads
)

target_compile_features(mighty_voxblox_core PUBLIC cxx_std_17)
