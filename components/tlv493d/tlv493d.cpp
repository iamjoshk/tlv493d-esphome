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
  uint8_t config[3];
  config[0] = 0x05;            // MOD1: Low Power Mode, 10Hz
  config[1] = factory_data[8]; // MOD2: Reserved Factory bits
  config[2] = factory_data[9]; // MOD3: Reserved Factory bits

  // 3. Calculate Parity (Critical for A1B6)
  // The FP (Frame Parity) bit is Bit 7 of the first config byte.
  // It must be the Even Parity (XOR) of all bits in the 3-byte config.
  uint8_t parity = 0;
  for (int i = 0; i < 3; i++) {
    for (int bit = 0; bit < 8; bit++) {
      if (i == 0 && bit == 7) continue; // Skip the parity bit itself
      if ((config[i] >> bit) & 0x01) parity++;
    }
  }
  if (parity % 2 != 0) {
    config[0] |= 0x80; // Set Bit 7 to 1 to achieve even parity
  }

  // 4. Send Wake-up Command
  if (!this->write(config, 3)) {
    ESP_LOGE(TAG, "Wake-up command failed (NACK)!");
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "TLV493D initialized with parity bit: %s", (config[0] & 0x80) ? "1" : "0");
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
  uint8_t data[7];
  // Read 7 bytes to get X, Y, Z, Temp, and the Register 4/5 bits
  if (!this->read_bytes(0x00, data, 7)) {
    ESP_LOGW(TAG, "Read failed!");
    return;
  }

  // Data Map for TLV493D-A1B6:
  // Reg 0: Bx [11:4] | Reg 1: By [11:4] | Reg 2: Bz [11:4]
  // Reg 4: Bx [3:0] (High) + By [3:0] (Low)
  // Reg 5: Temp [3:0] (High) + Bz [3:0] (Low)

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
