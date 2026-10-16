# The board's code here, and from the repository the servo loop
# (firmware/core), the link protocol (src/device/proto.cpp) that the
# host's tuushin library speaks, and the noise tax the books share with it
# (src/control/noise_tax.cpp).
set(repo ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(srcs
  main.cpp
  link.cpp
  calibration_store.cpp
  ${repo}/firmware/core/src/core.cpp
  ${repo}/firmware/core/src/foc.cpp
  ${repo}/src/device/proto.cpp
  ${repo}/src/control/noise_tax.cpp
)
if(CONFIG_TUUSHIN_EMULATED)
  list(APPEND srcs emulated_hal.cpp ${repo}/firmware/core/src/plant.cpp)
else()
  list(APPEND srcs board_hal.cpp bridge.cpp encoder.cpp supply.cpp)
endif()

idf_component_register(
  SRCS ${srcs}
  INCLUDE_DIRS . ${repo}/firmware/core/include ${repo}/include
  REQUIRES esp_driver_gpio esp_driver_mcpwm esp_driver_ledc esp_driver_spi esp_driver_i2c
           esp_driver_uart esp_driver_usb_serial_jtag esp_driver_gptimer esp_adc esp_timer
           nvs_flash
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wshadow)
