#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // 1. Read factory data (10 bytes)
  uint8_t factory_data[10];
  if (!this->read_bytes(0x00, factory_data, 10)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  // 2. Prepare Configuration
  // Use datarate mapping here; default is 0x05 low-power 10Hz.
  // The TLV493D datasheet indicates byte0 MOD1 + parity, byte1=MOD2, byte2=MOD3.
  this->config_[0] = 0x05;
  // Keep mod2/mod3 only low nibble from factory value (reserved bits), as per datasheet.
  this->config_[1] = factory_data[8] & 0x0F;
  this->config_[2] = factory_data[9] & 0x0F;

  // Data rate should be selectable via set_datarate (future expansion). A mapping
  // can be added here once the exact MOD1 bit encoding is confirmed.

  // 3. Calculate Parity (Critical for A1B6)
  // The FP (Frame Parity) bit is Bit 7 of the first config byte
  // and must be even parity of the 23 other bits.
  uint8_t parity = 0;
  for (int i = 0; i < 3; i++) {
    for (int bit = 0; bit < 8; bit++) {
      if (i == 0 && bit == 7) continue; // Skip parity bit.
      if ((this->config_[i] >> bit) & 0x01) parity++;
    }
  }
  if (parity % 2 != 0) {
    this->config_[0] |= 0x80; // Set Frame Parity bit
  }

  // 4. Send Wake-up/Mode Set command
  if (!this->write(this->config_, 3)) {
    ESP_LOGE(TAG, "Wake-up command failed (NACK)!");
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "TLV493D initialized with parity bit: %s", (this->config_[0] & 0x80) ? "1" : "0");
}

void TLV493DComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TLV493D:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
}

float TLV493DComponent::get_setup_priority() const { 
  return setup_priority::DATA; 
}

void TLV493DComponent::update() {
  uint8_t data[6];
  // Read 6 bytes to get X, Y, Z, status and low nibbles.
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Read failed!");
    return;
  }

  // Data Map for TLV493D-A1B6:
  // Reg 0: Bx [11:4]
  // Reg 1: By [11:4]
  // Reg 2: Bz [11:4]
  // Reg 3: status flags
  // Reg 4: Bx [3:0] (High nibble) + By [3:0] (Low nibble)
  // Reg 5: Temp [3:0] (High nibble) + Bz [3:0] (Low nibble)

  const uint8_t status = data[3];
  const bool data_ready = (status & 0x10) != 0;
  const bool of_x = (status & 0x01) != 0;
  const bool of_y = (status & 0x02) != 0;
  const bool of_z = (status & 0x04) != 0;

  ESP_LOGV(TAG, "Status byte=0x%02X (DRDY=%d, OVF x=%d,y=%d,z=%d)", status, data_ready, of_x, of_y, of_z);

  if (!data_ready) {
    ESP_LOGD(TAG, "TLV493D data not ready, skipping publish");
    return;
  }

  if (of_x || of_y || of_z) {
    ESP_LOGW(TAG, "Magnetometer overflow detected (status=0x%02X)", status);
  }

  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; // Sign extend

  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

  // Convert to uT (Sensitivity is 0.098 mT/LSB -> 98.0 uT/LSB)
  float x = raw_x * 98.0f;
  float y = raw_y * 98.0f;
  float z = raw_z * 98.0f;

  if (this->x_sensor_ != nullptr) this->x_sensor_->publish_state(x);
  if (this->y_sensor_ != nullptr) this->y_sensor_->publish_state(y);
  if (this->z_sensor_ != nullptr) this->z_sensor_->publish_state(z);

  if (this->heading_sensor_ != nullptr) {
    float heading = atan2f(y, x) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    this->heading_sensor_->publish_state(heading);
  }

  ESP_LOGV(TAG, "Raw Hex: %02X %02X %02X %02X %02X %02X", 
           data[0], data[1], data[2], data[3], data[4], data[5]);
}

}  // namespace tlv493d
}  // namespace esphome
