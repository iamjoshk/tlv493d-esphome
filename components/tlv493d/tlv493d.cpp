#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // 1. Verify communication
  uint8_t test_byte;
  if (!this->read_byte(0x00, &test_byte)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  // 2. Read current registers to get factory trimming bits (Reg 7, 8, 9)
  // This is required to preserve calibration when we write back configuration
  uint8_t factory_data[10];
  if (!this->read_bytes(0x00, factory_data, 10)) {
      this->mark_failed();
      return;
  }

  // 3. Wake up the sensor (Initialization)
  // We need to write to the MOD1 and MOD2 registers to start the ADC.
  // Byte 0: Reserved (0x00)
  // Byte 1: MOD1 (0x05 = Low Power Mode, 10Hz, Interrupts enabled)
  // Byte 2: MOD2 (Preserve bits from factory Reg 8)
  // Byte 3: MOD3 (Preserve bits from factory Reg 9)
  uint8_t write_data[4];
  write_data[0] = 0x00;
  write_data[1] = 0x05;            // Enable measurement mode
  write_data[2] = factory_data[8]; // Reserved trimming
  write_data[3] = (factory_data[9] & 0x1F) | 0x40;

  if (!this->write_bytes(0x00, write_data, 4)) {
      ESP_LOGE(TAG, "Failed to send initialization command!");
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed();
      return;
  }

  ESP_LOGD(TAG, "TLV493D Initialized and Measuring.");
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
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Updating TLV493D failed!");
    return;
  }

  // 12-bit signed integer conversion for TLV493D-A1B6
  // X: Reg0 (8 bits MSB) + Reg4 high nibble (4 bits LSB)
  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; // Sign extend 12-bit to 16-bit

  // Y: Reg1 (8 bits MSB) + Reg4 low nibble (4 bits LSB)
  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  // Z: Reg2 (8 bits MSB) + Reg5 low nibble (4 bits LSB)
  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

  // Sensitivity: 0.098 mT/LSB. Convert to MicroTesla (uT) -> 98.0
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
  
  ESP_LOGV(TAG, "Raw: X=%d, Y=%d, Z=%d", raw_x, raw_y, raw_z);
}

}  // namespace tlv493d
}  // namespace esphome
