add_executable(tuushin_tests
  test_main.cpp
  test_deadband.cpp
  test_tdpa.cpp
  test_channel.cpp
  test_crypto.cpp
  test_legacy_digest.cpp
  test_reflection.cpp
  test_teleop1d.cpp
  test_wire.cpp
  test_udp.cpp
  test_clock_sync.cpp
  test_probe.cpp
  test_teleop_controller.cpp
  test_teleop_node.cpp
  test_teleop_stall.cpp
  test_teleop_auth.cpp
  test_rendezvous.cpp
)
target_link_libraries(tuushin_tests PRIVATE tuushin)
add_test(NAME tuushin_tests COMMAND tuushin_tests)
