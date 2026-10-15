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
  src/control/held_device.cpp
  src/control/predictive_pc.cpp
  src/control/reflection.cpp
  src/control/tdpa.cpp
  src/crypto/legacy_digest.cpp
  src/crypto/noise.cpp
  src/device/proto.cpp
  src/device/remote.cpp
  src/device/stream.cpp
  src/net/channel.cpp
  src/net/clock_sync.cpp
  src/net/hops.cpp
  src/net/paths.cpp
  src/net/probe.cpp
  src/net/rendezvous.cpp
  src/net/stun.cpp
  src/net/trace.cpp
  src/net/turn.cpp
  src/net/udp.cpp
  src/rt/thread.cpp
  src/sim/teleop1d.cpp
  src/teleop/controller.cpp
  src/teleop/device.cpp
  src/teleop/link.cpp
  src/teleop/node.cpp
  src/wire/packet.cpp
)
target_include_directories(tuushin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# The hop count is measured on a thread of its own (teleop/node.cpp).
find_package(Threads REQUIRED)
target_link_libraries(tuushin PUBLIC Threads::Threads)

# libsodium (ISC license) provides the primitives behind crypto/noise.hpp:
# X25519, ChaCha20-Poly1305 and SHA-256. Its headers stay out of ours, so
# it is a private dependency.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(SODIUM QUIET IMPORTED_TARGET libsodium>=1.0.18)
endif()
if(TARGET PkgConfig::SODIUM)
  target_link_libraries(tuushin PRIVATE PkgConfig::SODIUM)
else()
  find_path(SODIUM_INCLUDE_DIR sodium.h)
  find_library(SODIUM_LIBRARY NAMES sodium libsodium)
  if(NOT SODIUM_INCLUDE_DIR OR NOT SODIUM_LIBRARY)
    message(FATAL_ERROR "libsodium not found; install it (apt install libsodium-dev, "
                        "brew install libsodium, vcpkg install libsodium)")
  endif()
  target_include_directories(tuushin PRIVATE ${SODIUM_INCLUDE_DIR})
  target_link_libraries(tuushin PRIVATE ${SODIUM_LIBRARY})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(tuushin PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()

# The servo loop of a device's firmware (firmware/core), built for the host
# as well: tuushin-devsim runs it on a simulated motor, and the tests drive
# it. The ESP-IDF project in firmware/esp32 compiles the same sources.
add_library(tuushin_fw
  firmware/core/src/core.cpp
  firmware/core/src/foc.cpp
  firmware/core/src/plant.cpp
)
target_include_directories(tuushin_fw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/firmware/core/include>
)
target_link_libraries(tuushin_fw PUBLIC tuushin)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(tuushin_fw PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
endif()

if(TUUSHIN_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(TUUSHIN_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
