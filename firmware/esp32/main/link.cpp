#include "link.hpp"

#include "freertos/FreeRTOS.h"

#if CONFIG_TUUSHIN_LINK_UART
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif

namespace tuushin::esp32 {

#if CONFIG_TUUSHIN_LINK_UART

namespace {
constexpr uart_port_t kPort = static_cast<uart_port_t>(CONFIG_TUUSHIN_UART_NUM);
}  // namespace

bool Link::begin() {
  uart_config_t cfg = {};
  cfg.baud_rate = CONFIG_TUUSHIN_UART_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  // A receive queue of a few hundred commands; no send queue (see above).
  if (uart_driver_install(kPort, 2048, 0, 0, nullptr, 0) != ESP_OK) return false;
  if (uart_param_config(kPort, &cfg) != ESP_OK) return false;
  if (uart_set_pin(kPort, CONFIG_TUUSHIN_UART_TX_GPIO, CONFIG_TUUSHIN_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }
  // A frame delimiter first: whatever the ROM printed at reset ends there,
  // and the host's reader drops it as one bad frame.
  const std::uint8_t zero = 0;
  write(&zero, 1);
  flush();
  return true;
}

std::size_t Link::read(std::uint8_t* buf, std::size_t cap) {
  const int n = uart_read_bytes(kPort, buf, static_cast<std::uint32_t>(cap), 0);
  return n > 0 ? static_cast<std::size_t>(n) : 0;
}

std::size_t Link::write(const std::uint8_t* buf, std::size_t n) {
  const std::size_t room = kRing - (head_ - tail_);
  if (n > room) n = room;
  for (std::size_t i = 0; i < n; ++i) ring_[(head_ + i) & (kRing - 1)] = buf[i];
  head_ += n;
  return n;
}

void Link::flush() {
  while (head_ != tail_) {
    const std::size_t at = tail_ & (kRing - 1);
    std::size_t run = head_ - tail_;
    if (run > kRing - at) run = kRing - at;  // up to the ring's end
    const int k = uart_tx_chars(kPort, reinterpret_cast<const char*>(ring_ + at),
                                static_cast<std::uint32_t>(run));
    if (k <= 0) break;
    tail_ += static_cast<std::size_t>(k);
    if (static_cast<std::size_t>(k) < run) break;  // the FIFO is full
  }
}

#else  // USB Serial/JTAG

bool Link::begin() {
  // The driver queues both ways and its calls take a timeout, 0 here.
  usb_serial_jtag_driver_config_t cfg = {};
  cfg.tx_buffer_size = 4096;
  cfg.rx_buffer_size = 2048;
  if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) return false;
  const std::uint8_t zero = 0;
  write(&zero, 1);
  return true;
}

std::size_t Link::read(std::uint8_t* buf, std::size_t cap) {
  const int n = usb_serial_jtag_read_bytes(buf, static_cast<std::uint32_t>(cap), 0);
  return n > 0 ? static_cast<std::size_t>(n) : 0;
}

std::size_t Link::write(const std::uint8_t* buf, std::size_t n) {
  const int k = usb_serial_jtag_write_bytes(buf, n, 0);
  return k > 0 ? static_cast<std::size_t>(k) : 0;
}

void Link::flush() {}

#endif

}  // namespace tuushin::esp32
