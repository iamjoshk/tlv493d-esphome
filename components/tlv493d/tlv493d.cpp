#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // 1. Read current registers to get factory trimming bits (Reg 7, 8, 9)
  uint8_t factory_data[10];
  if (!this->read_bytes(0x00, factory_data, 10)) {
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
  }

  // 2. Prepare Configuration Bytes
  // The A1B6 expects a raw 4-byte write (No register address)
  uint8_t config[4];
  
  // MOD1: 0x05 = Master Controlled Mode (Sensor waits for I2C read to trigger next conversion)
  // We must calculate the parity bit (Bit 7)
  uint8_t mod1 = 0x05; 
  // Calculate Parity for MOD1 (Bit 7)
  // For 0x05 (00000101), there are two '1's. For even parity, Bit 7 is 0.
  config[0] = mod1; 

  // MOD2: Configuration from factory Reg 8
  config[1] = factory_data[8];

  // MOD3: Configuration from factory Reg 9
  config[2] = factory_data[9];

  // 3. Perform RAW Write (No register address)
  // We use this->write_bytes_raw to send data directly after the I2C address
  if (!this->parent_->write_bytes(this->address_, config, 3)) {
      ESP_LOGE(TAG, "Failed to send initialization command (Raw Write)!");
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
  }

  ESP_LOGD(TAG, "TLV493D Initialized in Master Controlled Mode.");
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
  // In Master Controlled Mode, reading the data registers triggers the next measurement
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Updating TLV493D failed!");
    return;
  }

  // 12-bit signed integer conversion
  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; 

  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

  // Sensitivity: 98.0 uT per LSB
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
