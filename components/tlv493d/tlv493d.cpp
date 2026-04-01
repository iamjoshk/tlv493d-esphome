#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // 1. Read factory trimming bits (Registers 7, 8, 9)
  // We read 10 bytes starting from 0x00.
  uint8_t factory_data[10];
  if (!this->read_bytes(0x00, factory_data, 10)) {
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
  }

  // 2. Prepare Configuration
  // Byte 0: MOD1, Byte 1: MOD2, Byte 2: MOD3
  uint8_t config[3];
  
  // MOD1: 0x05 
  // Master Controlled Mode, Low Power. 
  // Parity bit (Bit 7) must be calculated. For 0x05, parity is 0.
  config[0] = 0x05; 

  // MOD2: Factory Reserved (From Register 8)
  config[1] = factory_data[8];

  // MOD3: Factory Reserved (From Register 9)
  config[2] = factory_data[9];

  // 3. Perform RAW Write (No register address)
  // Fixed: Use 'write' instead of 'write_bytes_raw'
  if (!this->write(config, 3)) {
      ESP_LOGE(TAG, "Failed to send initialization command (NACK)!");
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
  }

  ESP_LOGD(TAG, "TLV493D Initialized successfully.");
}

void TLV493DComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TLV493D:");
  LOG_I2C_DEVICE(this);
  if (this->error_code_ == COMMUNICATION_FAILED) {
    ESP_LOGE(TAG, "Communication with TLV493D failed!");
  }
  LOG_UPDATE_INTERVAL(this);
}

float TLV493DComponent::get_setup_priority() const { 
  return setup_priority::DATA; 
}

void TLV493DComponent::update() {
  uint8_t data[6];
  // Reading 6 bytes triggers the next measurement in Master Controlled Mode
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Updating TLV493D failed!");
    return;
  }

  // 12-bit signed conversion for TLV493D-A1B6
  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; 

  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

  // Sensitivity: 98.0 uT per LSB (approx 0.098 mT)
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
