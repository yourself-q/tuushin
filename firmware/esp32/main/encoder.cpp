#include "encoder.hpp"

#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_TUUSHIN_ENCODER_AS5600
#include "driver/i2c_master.h"
#else
#include "driver/spi_master.h"
#endif

namespace tuushin::esp32 {

namespace {

#if CONFIG_TUUSHIN_ENCODER_AS5600

constexpr std::uint8_t kAddress = 0x36;
constexpr std::uint8_t kConf = 0x07;    // CONF, high byte first
constexpr std::uint8_t kStatus = 0x0B;  // STATUS, then RAW ANGLE high and low
constexpr std::uint8_t kMagnetDetected = 1u << 5;
constexpr std::uint8_t kMagnetWeak = 1u << 4;
constexpr std::uint8_t kMagnetStrong = 1u << 3;
constexpr int kTimeoutMs = 1;

i2c_master_bus_handle_t g_bus = nullptr;
i2c_master_dev_handle_t g_dev = nullptr;

#else

spi_device_handle_t g_dev = nullptr;

// One full-duplex frame of `bits` (up to 32) through the transaction's own
// buffers, no DMA.
bool transfer(std::uint32_t tx, int bits, std::uint32_t* rx) {
  spi_transaction_t t = {};
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.length = static_cast<std::size_t>(bits);
  const int bytes = (bits + 7) / 8;
  for (int i = 0; i < bytes; ++i) t.tx_data[i] = static_cast<std::uint8_t>(tx >> (8 * (bytes - 1 - i)));
  if (spi_device_polling_transmit(g_dev, &t) != ESP_OK) return false;
  std::uint32_t v = 0;
  for (int i = 0; i < bytes; ++i) v = (v << 8) | t.rx_data[i];
  *rx = v;
  return true;
}

#endif

#if CONFIG_TUUSHIN_ENCODER_MT6701

// CRC-6, x^6 + x + 1, first bit first, from 0: over the 18 bits of angle
// and status.
std::uint32_t crc6(std::uint32_t data, int bits) {
  std::uint32_t crc = 0;
  for (int i = bits - 1; i >= 0; --i) {
    const std::uint32_t in = ((data >> i) & 1u) ^ ((crc >> 5) & 1u);
    crc = (crc << 1) & 0x3Fu;
    if (in) crc ^= 0x03u;
  }
  return crc;
}

#endif

#if CONFIG_TUUSHIN_ENCODER_AS5047P

constexpr std::uint32_t kReadAngle = 0xFFFF;  // read ANGLECOM (0x3FFF), parity included
constexpr std::uint32_t kReadErrors = 0x4001;  // read ERRFL (0x0001), which clears it

bool even_parity(std::uint32_t w) { return (__builtin_popcount(w & 0xFFFFu) & 1) == 0; }

#endif

}  // namespace

bool Encoder::begin() {
  // The encoders take up to 10 ms from power to their first good reading.
  vTaskDelay(pdMS_TO_TICKS(20));
#if CONFIG_TUUSHIN_ENCODER_AS5600
  i2c_master_bus_config_t bus = {};
  bus.i2c_port = -1;  // any free one
  bus.sda_io_num = static_cast<gpio_num_t>(CONFIG_TUUSHIN_ENC_SDA_GPIO);
  bus.scl_io_num = static_cast<gpio_num_t>(CONFIG_TUUSHIN_ENC_SCL_GPIO);
  bus.clk_source = I2C_CLK_SRC_DEFAULT;
  bus.glitch_ignore_cnt = 7;
  // The breakout boards have their own pull-ups; these weak ones only
  // help a bare chip at 100 kHz.
  bus.flags.enable_internal_pullup = true;
  if (i2c_new_master_bus(&bus, &g_bus) != ESP_OK) return false;
  i2c_device_config_t dev = {};
  dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev.device_address = kAddress;
  dev.scl_speed_hz = CONFIG_TUUSHIN_ENC_I2C_HZ;
  if (i2c_master_bus_add_device(g_bus, &dev, &g_dev) != ESP_OK) return false;
  // CONF: the slow filter at 2x, no fast filter, no hysteresis, no
  // watchdog, normal power. Only the volatile register is written; the
  // chip's one-time memory is left alone.
  const std::uint8_t conf[3] = {kConf, 0x03, 0x00};
  if (i2c_master_transmit(g_dev, conf, sizeof conf, 10) != ESP_OK) return false;
#else
  spi_bus_config_t bus = {};
  bus.sclk_io_num = CONFIG_TUUSHIN_ENC_SCLK_GPIO;
  bus.miso_io_num = CONFIG_TUUSHIN_ENC_MISO_GPIO;
  bus.mosi_io_num = CONFIG_TUUSHIN_ENC_MOSI_GPIO;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 4;
  if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED) != ESP_OK) return false;
  spi_device_interface_config_t dev = {};
  dev.mode = 1;
  dev.clock_speed_hz = CONFIG_TUUSHIN_ENC_SPI_HZ;
  dev.spics_io_num = CONFIG_TUUSHIN_ENC_CS_GPIO;
  dev.queue_size = 1;
  // Both chips want CS down a while before the first edge and up a while
  // between frames (350 ns for the AS5047P): a few SPI clocks of each.
  dev.cs_ena_pretrans = 2;
  dev.cs_ena_posttrans = 2;
  if (spi_bus_add_device(SPI2_HOST, &dev, &g_dev) != ESP_OK) return false;
  // The bus is the encoder's alone: holding it saves the arbitration on
  // every transfer.
  if (spi_device_acquire_bus(g_dev, portMAX_DELAY) != ESP_OK) return false;
#if CONFIG_TUUSHIN_ENCODER_AS5047P
  // Clear whatever error the power-up left.
  std::uint32_t w;
  transfer(kReadErrors, 16, &w);
  transfer(kReadAngle, 16, &w);
#endif
#endif
  std::uint32_t raw;
  return read(&raw);
}

bool Encoder::read(std::uint32_t* raw) {
  if (read_once(raw)) return true;
  ++bad_;
  if (read_once(raw)) return true;
  ++bad_;
  return false;
}

bool Encoder::read_once(std::uint32_t* raw) {
  if (!g_dev) return false;  // it did not come up
#if CONFIG_TUUSHIN_ENCODER_MT6701
  std::uint32_t w;
  if (!transfer(0, 24, &w)) return false;
  const std::uint32_t angle_status = w >> 6;
  if (crc6(angle_status, 18) != (w & 0x3Fu)) return false;
  const std::uint32_t status = angle_status & 0xFu;
  if (status & 0x8u) return false;  // loss of track: the reading lags the magnet
  if (status & 0x3u) ++weak_;       // the field too strong (1) or too weak (2)
  *raw = angle_status >> 4;
  return true;
#elif CONFIG_TUUSHIN_ENCODER_AS5047P
  std::uint32_t w;
  // The first frame asks; the second's answer is the angle as the first
  // ended (and asks again, which nothing reads).
  if (!transfer(kReadAngle, 16, &w) || !transfer(kReadAngle, 16, &w)) return false;
  if (!even_parity(w) || (w & 0x4000u)) {
    // The flag stays up until the error register is read.
    transfer(kReadErrors, 16, &w);
    transfer(kReadAngle, 16, &w);
    return false;
  }
  *raw = w & 0x3FFFu;
  return true;
#else  // AS5600
  std::uint8_t r[3];
  if (i2c_master_transmit_receive(g_dev, &kStatus, 1, r, sizeof r, kTimeoutMs) != ESP_OK) return false;
  if (!(r[0] & kMagnetDetected)) return false;
  if (r[0] & (kMagnetWeak | kMagnetStrong)) ++weak_;
  *raw = ((static_cast<std::uint32_t>(r[1]) << 8) | r[2]) & 0x0FFFu;
  return true;
#endif
}

}  // namespace tuushin::esp32
