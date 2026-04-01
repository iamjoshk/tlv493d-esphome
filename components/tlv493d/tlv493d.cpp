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

  ESP_LOGD(TAG, "TLV493D factory bytes[7..9]: %02X %02X %02X", factory_data[7], factory_data[8], factory_data[9]);

  // 2. Prepare Configuration for Master Controlled Mode (MCM).
  // In MCM (FAST=1, LOWPOWER=1) the sensor takes a fresh measurement on every
  // I2C read, so no data-ready polling is needed.
  //
  // Register layout (TLV493D-A1B6 User Manual, Table 2):
  //   MOD1 = config[0]: FP(7) | ADDR(6:5) | RES1(4:3) | INT(2) | FAST(1) | LOWPOWER(0)
  //   MOD2 = config[1]: factory_data[8] (must be preserved verbatim)
  //   MOD3 = config[2]: factory_data[9] bits [4:0] (lower 5 bits only)
  uint8_t res1_bits = (factory_data[7] & 0x18);  // bits 4:3, keep in position
  this->config_[0] = res1_bits | 0x03;            // RES1 | FAST=1 | LOWPOWER=1
  this->config_[1] = factory_data[8];             // MOD2: reserved, must be preserved
  this->config_[2] = factory_data[9] & 0x1F;      // MOD3: reserved bits [4:0] only

  // 3. Compute even parity over all 23 non-parity bits and set bit 7 of config[0].
  uint8_t parity = 0;
  for (int i = 0; i < 3; i++) {
    for (int bit = 0; bit < 8; bit++) {
      if (i == 0 && bit == 7) continue;  // skip parity bit itself
      if ((this->config_[i] >> bit) & 0x01) parity++;
    }
  }
  if (parity % 2 != 0) {
    this->config_[0] |= 0x80;  // set FP to achieve even parity
  }

  ESP_LOGD(TAG, "TLV493D config: %02X %02X %02X (parity=%s)",
           this->config_[0], this->config_[1], this->config_[2],
           (this->config_[0] & 0x80) ? "1" : "0");

  // 4. Write configuration: 3-byte raw write (MOD1, MOD2, MOD3) with no register
  //    address prefix byte. A leading 0x00 would be parsed by the sensor as
  //    MOD1=0x00 (power-down), so write() must be used here, not write_bytes().
  bool write_ok = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (this->write(this->config_, 3)) {
      write_ok = true;
      break;
    }
    ESP_LOGW(TAG, "Config write attempt %d NACK, retrying...", attempt);
  }
  if (!write_ok) {
    // Fallback: zero out reserved bytes, keep MOD1 (mode + parity).
    uint8_t fallback[3] = {this->config_[0], 0x00, 0x00};
    ESP_LOGW(TAG, "Trying fallback config: %02X 00 00", fallback[0]);
    if (!this->write(fallback, 3)) {
      ESP_LOGE(TAG, "Configuration write failed (NACK) after retries!");
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
    }
  }

  ESP_LOGD(TAG, "TLV493D setup complete (Master Controlled Mode)");
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

  // Data Map for TLV493D-A1B6 (User Manual Table 1):
  // Byte 0: Bx[11:4]
  // Byte 1: By[11:4]
  // Byte 2: Bz[11:4]
  // Byte 3: CH[3:2] FRML[1:0]  (frame counter & channel, NOT a data-ready flag)
  // Byte 4: Bx[3:0] (high nibble) | By[3:0] (low nibble)
  // Byte 5: PWDN(4) | Temp[3:0] (high nibble) | Bz[3:0] (low nibble)
  //
  // In Master Controlled Mode every read returns a freshly taken measurement,
  // so no data-ready polling is required.

  // Optionally warn if sensor reports power-down state (bit 4 of byte 5).
  if (data[5] & 0x10) {
    ESP_LOGW(TAG, "TLV493D power-down flag set (byte5=0x%02X) — data may be stale", data[5]);
  }

  ESP_LOGV(TAG, "Raw: %02X %02X %02X %02X %02X %02X (frame/ch=0x%02X)",
           data[0], data[1], data[2], data[3], data[4], data[5], data[3]);

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
}

}  // namespace tlv493d
}  // namespace esphome
