cmake_minimum_required(VERSION 3.16)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES AND EXISTS "/opt/homebrew")
  set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "macOS target architecture" FORCE)
endif()

project(mighty_cpp_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_executable(mightyapp main.cpp)

target_include_directories(mightyapp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp
)

target_link_libraries(mightyapp PRIVATE ${OpenCV_LIBS})
