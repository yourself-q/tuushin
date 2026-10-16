add_executable(tuushin_tests
  test_main.cpp
  test_deadband.cpp
  test_bounded_lp.cpp
  test_count_bounds.cpp
  test_tdpa.cpp
  test_reading_noise.cpp
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
  test_stun.cpp
  test_turn.cpp
  test_hops.cpp
  test_device_proto.cpp
  test_fw_core.cpp
  test_device_remote.cpp
)
target_link_libraries(tuushin_tests PRIVATE tuushin_fw)
add_test(NAME tuushin_tests COMMAND tuushin_tests)
# TURN over TLS, against a TLS front of the tests' own (sim_tls.hpp).
if(TUUSHIN_HAVE_TLS)
  target_sources(tuushin_tests PRIVATE test_tls.cpp)
  target_link_libraries(tuushin_tests PRIVATE OpenSSL::SSL OpenSSL::Crypto)
endif()
