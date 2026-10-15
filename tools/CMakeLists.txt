add_executable(tuushin-sim tuushin_sim.cpp)
target_link_libraries(tuushin-sim PRIVATE tuushin)

add_executable(tuushin-probe tuushin_probe.cpp)
target_link_libraries(tuushin-probe PRIVATE tuushin)

add_executable(tuushin-teleop tuushin_teleop.cpp)
target_link_libraries(tuushin-teleop PRIVATE tuushin)

add_executable(tuushin-devsim tuushin_devsim.cpp)
target_link_libraries(tuushin-devsim PRIVATE tuushin_fw)

# tuushin-netreplay replays traces on a Linux interface with an eBPF
# program, so it needs Linux, clang (for the BPF target) and libbpf, which
# is offered under LGPL-2.1 or BSD-2-Clause and used here under the latter.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_program(TUUSHIN_BPF_CLANG NAMES clang clang-19 clang-18 clang-17 clang-16 clang-15 clang-14)
  if(PkgConfig_FOUND)
    pkg_check_modules(LIBBPF QUIET IMPORTED_TARGET libbpf>=0.8)
  endif()
  if(TUUSHIN_BPF_CLANG AND TARGET PkgConfig::LIBBPF)
    set(bpf_object ${CMAKE_CURRENT_BINARY_DIR}/netreplay.bpf.o)
    set(bpf_source ${CMAKE_CURRENT_BINARY_DIR}/netreplay_bpf.cpp)
    # The BPF target has no multiarch include directory of its own, where
    # Debian and Ubuntu keep asm/types.h.
    set(bpf_includes)
    if(CMAKE_LIBRARY_ARCHITECTURE)
      list(APPEND bpf_includes -I/usr/include/${CMAKE_LIBRARY_ARCHITECTURE})
    endif()
    foreach(dir ${LIBBPF_INCLUDE_DIRS})
      list(APPEND bpf_includes -I${dir})
    endforeach()
    add_custom_command(
      OUTPUT ${bpf_object}
      COMMAND ${TUUSHIN_BPF_CLANG} -O2 -g -target bpf -Wall ${bpf_includes}
              -c ${CMAKE_CURRENT_SOURCE_DIR}/netreplay/netreplay.bpf.c -o ${bpf_object}
      DEPENDS netreplay/netreplay.bpf.c netreplay/netreplay.h
      COMMENT "Compiling the netreplay eBPF program"
      VERBATIM)
    add_custom_command(
      OUTPUT ${bpf_source}
      COMMAND ${CMAKE_COMMAND} -DINPUT=${bpf_object} -DOUTPUT=${bpf_source}
              -P ${CMAKE_CURRENT_SOURCE_DIR}/netreplay/embed.cmake
      DEPENDS ${bpf_object} netreplay/embed.cmake
      VERBATIM)
    add_executable(tuushin-netreplay tuushin_netreplay.cpp ${bpf_source})
    target_include_directories(tuushin-netreplay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/netreplay)
    target_link_libraries(tuushin-netreplay PRIVATE tuushin PkgConfig::LIBBPF)
  else()
    message(STATUS "tuushin-netreplay not built: it needs clang and libbpf (apt install clang libbpf-dev)")
  endif()
endif()
