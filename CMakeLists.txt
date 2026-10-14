cmake_minimum_required(VERSION 3.16)

project(tuushin VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(TUUSHIN_BUILD_TESTS "Build the unit tests" ON)
option(TUUSHIN_BUILD_TOOLS "Build the command-line tools" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(tuushin
  src/time.cpp
  src/control/predictive_pc.cpp
  src/control/reflection.cpp
  src/control/tdpa.cpp
  src/net/channel.cpp
  src/net/clock_sync.cpp
  src/net/probe.cpp
  src/net/trace.cpp
  src/net/udp.cpp
  src/rt/thread.cpp
  src/sim/teleop1d.cpp
  src/teleop/controller.cpp
  src/teleop/device.cpp
  src/wire/packet.cpp
)
target_include_directories(tuushin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(tuushin PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()

if(TUUSHIN_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(TUUSHIN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
