add_executable(tuushin-sim tuushin_sim.cpp)
target_link_libraries(tuushin-sim PRIVATE tuushin)

add_executable(tuushin-probe tuushin_probe.cpp)
target_link_libraries(tuushin-probe PRIVATE tuushin)

add_executable(tuushin-teleop tuushin_teleop.cpp)
target_link_libraries(tuushin-teleop PRIVATE tuushin)
