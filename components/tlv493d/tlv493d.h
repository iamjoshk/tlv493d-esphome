#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace tlv493d {

enum TLV493DDatarate {
  TLV493D_DATARATE_75_0_HZ,
  TLV493D_DATARATE_150_0_HZ,
  TLV493D_DATARATE_255_0_HZ,
};

class TLV493DComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_datarate(TLV493DDatarate datarate) { datarate_ = datarate; }
  void set_x_sensor(sensor::Sensor *x_sensor) { x_sensor_ = x_sensor; }
  void set_y_sensor(sensor::Sensor *y_sensor) { y_sensor_ = y_sensor; }
  void set_z_sensor(sensor::Sensor *z_sensor) { z_sensor_ = z_sensor; }
  void set_heading_sensor(sensor::Sensor *heading_sensor) { heading_sensor_ = heading_sensor; }
  void set_magnitude_sensor(sensor::Sensor *magnitude_sensor) { magnitude_sensor_ = magnitude_sensor; }

 protected:
  TLV493DDatarate datarate_{TLV493D_DATARATE_75_0_HZ};
  uint8_t config_[3] = {0x05, 0x00, 0x00};
  uint8_t last_frame_counter_{0xFF};  // 0xFF = sentinel (no previous read)
  sensor::Sensor *x_sensor_{nullptr};
  sensor::Sensor *y_sensor_{nullptr};
  sensor::Sensor *z_sensor_{nullptr};
  sensor::Sensor *heading_sensor_{nullptr};
  sensor::Sensor *magnitude_sensor_{nullptr};
  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    ID_REGISTERS,
  } error_code_;
};

}  // namespace tlv493d
}  // namespace esphome
