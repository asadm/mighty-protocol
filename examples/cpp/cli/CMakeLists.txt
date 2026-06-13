cmake_minimum_required(VERSION 3.16)

project(mighty_cli_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(mighty_cli main.cpp)

target_include_directories(mighty_cli PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp
)

if(UNIX AND NOT APPLE)
  target_link_libraries(mighty_cli PRIVATE pthread)
endif()
