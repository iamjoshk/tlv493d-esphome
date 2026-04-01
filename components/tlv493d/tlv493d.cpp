#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // FIXED: We must provide a real variable, not nullptr
  uint8_t test_byte;
  if (!this->read_byte(0x00, &test_byte)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Communication successful, received: 0x%02X", test_byte);
}

void TLV493DComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TLV493D:");
  LOG_I2C_DEVICE(this);
  if (this->error_code_ == COMMUNICATION_FAILED) {
    ESP_LOGE(TAG, "Communication with TLV493D failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "X Axis", this->x_sensor_);
  LOG_SENSOR("  ", "Y Axis", this->y_sensor_);
  LOG_SENSOR("  ", "Z Axis", this->z_sensor_);
  LOG_SENSOR("  ", "Heading", this->heading_sensor_);
}

float TLV493DComponent::get_setup_priority() const { 
  return setup_priority::DATA; 
}

void TLV493DComponent::update() {
  uint8_t data[6];
  // Note: The TLV493D often requires reading from the start of the register map
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Updating TLV493D failed!");
    return;
  }

  // Conversion logic for TLV493D-A1B6 (12-bit)
  // X: 8 bits from Reg 0 + top 4 bits from Reg 4
  int16_t raw_x = (int16_t)((data[0] << 8) | (data[4] & 0xF0)) >> 4;
  // Y: 8 bits from Reg 1 + bottom 4 bits from Reg 4
  int16_t raw_y = (int16_t)((data[1] << 8) | ((data[4] & 0x0F) << 4)) >> 4;
  // Z: 8 bits from Reg 2 + bottom 4 bits from Reg 5
  int16_t raw_z = (int16_t)((data[2] << 8) | ((data[5] & 0x0F) << 4)) >> 4;

  // Sensitivity is approx 0.098 mT per LSB
  // We convert to MicroTesla (uT) as per your Python config
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
