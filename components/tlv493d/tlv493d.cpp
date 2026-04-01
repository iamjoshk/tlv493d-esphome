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

  // 2. Build write buffer for Master Controlled Mode (MCM: FAST=1, LOWPOWER=1).
  //
  // The TLV493D-A1B6 requires a 4-byte I2C write (per Adafruit + Infineon protocol):
  //   Wire byte 0: 0x00     (always zero — leading protocol byte, carries no config)
  //   Wire byte 1: config[0] = FP(7)=1 | ADDR(6:5) | RES1(4:3) | INT(2)=0 | FAST(1)=1 | LP(0)=1
  //   Wire byte 2: config[1] = RES2 = factory_data[8] verbatim
  //   Wire byte 3: config[2] = RES3 = factory_data[9] bits[4:0]
  //
  // write_bytes(0x00, config_, 3) sends exactly [0x00, config_[0], config_[1], config_[2]].
  // FP (bit 7 of wire byte 1) is set to 1, matching Adafruit and avjui reference implementations.
  this->config_[0] = (factory_data[7] & 0x18) | 0x83;  // RES1 | FP=1 | FAST=1 | LOWPOWER=1
  this->config_[1] = factory_data[8];                    // RES2: must be preserved verbatim
  this->config_[2] = factory_data[9] & 0x1F;             // RES3: lower 5 bits only

  ESP_LOGD(TAG, "TLV493D config: %02X %02X %02X", this->config_[0], this->config_[1], this->config_[2]);

  // 3. Write 4-byte configuration frame.
  if (!this->write_bytes(0x00, this->config_, 3)) {
    ESP_LOGE(TAG, "Configuration write failed (NACK)!");
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
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
  // The TLV493D requires a preceding 0x00 register-address byte before every read.
  // Pure read() with 0 write bytes NACKs on this device via ESPHome's I2C IDF backend.
  // read_bytes(0x00, ...) sends [WRITE 0x00][READ], which the sensor ACKs and returns data.
  if (!this->read_bytes(0x00, data, 6)) {
    ESP_LOGW(TAG, "Read failed!");
    return;
  }

  // Data Map for TLV493D-A1B6 (User Manual Table 1):
  // Byte 0: Bx[11:4]
  // Byte 1: By[11:4]
  // Byte 2: Bz[11:4]
  // Byte 3: TEMP1[7:4] | FRAMECOUNTER[3:2] | CHANNEL[1:0]
  // Byte 4: Bx[3:0] (high nibble) | By[3:0] (low nibble)
  // Byte 5: flags[7:4] | Bz[3:0] (low nibble)
  //
  // In Master Controlled Mode every pure read triggers and returns a fresh measurement.

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
