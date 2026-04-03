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

  // 2. Build write buffer for Low Power mode (autonomous, ~10 Hz).
  //
  // MCM (FAST=1, LOWPOWER=1) requires a pure I2C START to trigger each conversion.
  // ESPHome's read_bytes() issues a repeated-start (not a fresh START), which the
  // sensor does not treat as an MCM trigger — leaving FRAMECOUNTER frozen.
  // Low Power mode (FAST=0, LOWPOWER=1) measures autonomously at ~10 Hz so
  // ESPHome can simply read the latest result whenever it polls.
  //
  // 4-byte write frame: [0x00, MOD1, RES2, RES3]
  //   MOD1: FP(7)=1 | ADDR(6:5)=0 | RES1(4:3) | INT(2)=0 | FAST(1)=0 | LP(0)=1
  this->config_[0] = (factory_data[7] & 0x18) | 0x81;   // RES1 | FP=1 | FAST=0 | LOWPOWER=1
  this->config_[1] = factory_data[8];                    // RES2: must be preserved verbatim
  this->config_[2] = (factory_data[9] & 0x1F) | 0x40;   // RES3 | LP_Period=1 (bit 6) → 12ms cycle (~83 Hz)

  ESP_LOGD(TAG, "TLV493D config: %02X %02X %02X", this->config_[0], this->config_[1], this->config_[2]);

  // 3. Write 4-byte configuration frame.
  if (!this->write_bytes(0x00, this->config_, 3)) {
    ESP_LOGE(TAG, "Configuration write failed (NACK)!");
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "TLV493D setup complete (Low Power autonomous mode)");
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
  // Read 7 bytes to get X, Y, Z, status, low nibbles, and temperature.
  // The TLV493D requires a preceding 0x00 register-address byte before every read.
  // Pure read() with 0 write bytes NACKs on this device via ESPHome's I2C IDF backend.
  // read_bytes(0x00, ...) sends [WRITE 0x00][READ], which the sensor ACKs and returns data.
  if (!this->read_bytes(0x00, data, 7)) {
    ESP_LOGW(TAG, "Read failed!");
    return;
  }

  // Data Map for TLV493D-A1B6 (User Manual Table 1):
  // Byte 0: Bx[11:4]
  // Byte 1: By[11:4]
  // Byte 2: Bz[11:4]
  // Byte 3: T[11:8] (high nibble) | FRAMECOUNTER[3:2] | CHANNEL[1:0]
  // Byte 4: Bx[3:0] (high nibble) | By[3:0] (low nibble)
  // Byte 5: PD_FLAG (bit 4) | Bz[3:0] (low nibble)
  // Byte 6: T[7:0]

  // Check FRAMECOUNTER (bits 3:2 of byte 3) to detect stale data.
  uint8_t frame = (data[3] & 0x0C) >> 2;
  if (frame == this->last_frame_counter_) {
    ESP_LOGV(TAG, "Stale data (FC=%d unchanged), skipping", frame);
    return;
  }
  this->last_frame_counter_ = frame;

  ESP_LOGV(TAG, "Raw: %02X %02X %02X %02X %02X %02X %02X (frame/ch=0x%02X)",
           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[3]);

  int16_t raw_x = (int16_t)((data[0] << 4) | (data[4] >> 4));
  if (raw_x & 0x0800) raw_x |= 0xF000; // Sign extend

  int16_t raw_y = (int16_t)((data[1] << 4) | (data[4] & 0x0F));
  if (raw_y & 0x0800) raw_y |= 0xF000;

  int16_t raw_z = (int16_t)((data[2] << 4) | (data[5] & 0x0F));
  if (raw_z & 0x0800) raw_z |= 0xF000;

  // Temperature: T[11:8] in byte 3 high nibble (R_TEMP1), T[7:0] in byte 6 (R_TEMP2).
  // Per TLV493D-A1B6 datasheet: TRAW = TOFFSET + TSENS * TMEAS
  //   TSENS = 1.1 LSB/°C. TOFFSET empirically determined as 349.
  //   Solving for TMEAS: T(°C) = (TRAW - 349) / 1.1
  int16_t raw_t = (int16_t)(((data[3] & 0xF0) << 4) | data[6]);
  float temp_c = (raw_t - 349.0f) / 1.1f;

  // Convert to uT (Sensitivity is 0.098 mT/LSB -> 98.0 uT/LSB)
  float x = raw_x * 98.0f;
  float y = raw_y * 98.0f;
  float z = raw_z * 98.0f;

  // Apply EMA smoothing when configured (alpha > 0). Raw passthrough when alpha == 0.
  if (this->ema_alpha_ > 0.0f) {
    if (std::isnan(this->ema_x_)) {
      this->ema_x_ = x;
      this->ema_y_ = y;
      this->ema_z_ = z;
    } else {
      this->ema_x_ = this->ema_alpha_ * x + (1.0f - this->ema_alpha_) * this->ema_x_;
      this->ema_y_ = this->ema_alpha_ * y + (1.0f - this->ema_alpha_) * this->ema_y_;
      this->ema_z_ = this->ema_alpha_ * z + (1.0f - this->ema_alpha_) * this->ema_z_;
    }
  } else {
    this->ema_x_ = x;
    this->ema_y_ = y;
    this->ema_z_ = z;
  }

  if (this->x_sensor_ != nullptr) this->x_sensor_->publish_state(this->ema_x_);
  if (this->y_sensor_ != nullptr) this->y_sensor_->publish_state(this->ema_y_);
  if (this->z_sensor_ != nullptr) this->z_sensor_->publish_state(this->ema_z_);

  if (this->heading_sensor_ != nullptr) {
    float heading = atan2f(this->ema_y_, this->ema_x_) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    this->heading_sensor_->publish_state(heading);
  }

  if (this->magnitude_sensor_ != nullptr) {
    float magnitude = sqrtf(this->ema_x_ * this->ema_x_ + this->ema_y_ * this->ema_y_ + this->ema_z_ * this->ema_z_);
    this->magnitude_sensor_->publish_state(magnitude);
  }

  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temp_c);
  }
}

}  // namespace tlv493d
}  // namespace esphome
