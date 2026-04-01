#include "tlv493d.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace tlv493d {

static const char *const TAG = "tlv493d";

void TLV493DComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TLV493D...");
  
  // 1. I2C Interface Reset
  // The A1B6 can get stuck in a 1-byte read mode. Sending a 0xFF 
  // command helps reset the internal I2C state machine.
  this->write(nullptr, 0); 

  // 2. Read factory data
  uint8_t factory_data[10];
  if (!this->read_bytes(0x00, factory_data, 10)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  // 3. Prepare 4-byte Configuration Frame for A1B6
  // Byte 0: Reserved (0x00)
  // Byte 1: MOD1 (0x01 = Low Power Mode, 0x05 = Master Controlled)
  // Byte 2: MOD2 (Reserved bits)
  // Byte 3: MOD3 (Reserved bits)
  uint8_t config[4];
  config[0] = 0x00; 
  config[1] = 0x01; // Low Power Mode (Free-running)
  config[2] = factory_data[8];
  config[3] = (factory_data[9] & 0x1F) | 0x40; // Set high bits for A1B6 compatibility

  // 4. Parity Calculation (Even Parity)
  // The A1B6 calculates parity over all written bits.
  // The parity bit is stored in MOD1 (config[1]) Bit 7.
  uint8_t parity = 0;
  for (int i = 0; i < 4; i++) {
    for (int bit = 0; bit < 8; bit++) {
      if (i == 1 && bit == 7) continue; // Skip the parity bit slot
      if ((config[i] >> bit) & 0x01) parity++;
    }
  }
  if (parity % 2 != 0) {
    config[1] |= 0x80; // Set Parity Bit to 1
  }

  // 5. Send Configuration (Raw write, no register address)
  if (!this->write(config, 4)) {
    ESP_LOGE(TAG, "Failed to wake up sensor!");
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Wake-up command sent. Parity: %d", (config[1] >> 7));
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
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "I2C Read failed");
    return;
  }

  // Check the Power Down bit (Reg 5, bit 4)
  if (data[5] & 0x10) {
    ESP_LOGW(TAG, "Sensor is still in Power Down mode! (Reg5: %02X)", data[5]);
  }

  // 12-bit signed conversion
  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; 

  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

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