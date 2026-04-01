import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ADDRESS,
    CONF_FIELD_STRENGTH_X,
    CONF_FIELD_STRENGTH_Y,
    CONF_FIELD_STRENGTH_Z,
    CONF_HEADING,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_MAGNET,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_MICROTESLA,
    UNIT_DEGREES,
    ICON_SCREEN_ROTATION,
    CONF_UPDATE_INTERVAL,
)

DEPENDENCIES = ["i2c"]

tlv493d_ns = cg.esphome_ns.namespace("tlv493d")

TLV493DComponent = tlv493d_ns.class_(
    "TLV493DComponent", cg.PollingComponent, i2c.I2CDevice
)

TLV493DDatarate = tlv493d_ns.enum("TLV493DDatarate")
TLV493DDatarates = {
    75: TLV493DDatarate.TLV493D_DATARATE_75_0_HZ,
    150: TLV493DDatarate.TLV493D_DATARATE_150_0_HZ,
    255: TLV493DDatarate.TLV493D_DATARATE_255_0_HZ,
}

CONF_MAGNITUDE = "magnitude"
CONF_OVERSAMPLING = "oversampling"  # accepted but unused; for compatibility with tronikos framework
CONF_SMOOTHING_FACTOR = "smoothing_factor"

field_strength_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_MICROTESLA,
    icon=ICON_MAGNET,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
heading_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=1,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    icon=ICON_THERMOMETER,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
    device_class=DEVICE_CLASS_TEMPERATURE,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TLV493DComponent),
            cv.Optional(CONF_ADDRESS): cv.i2c_address,
            cv.Optional(CONF_FIELD_STRENGTH_X): field_strength_schema,
            cv.Optional(CONF_FIELD_STRENGTH_Y): field_strength_schema,
            cv.Optional(CONF_FIELD_STRENGTH_Z): field_strength_schema,
            cv.Optional(CONF_HEADING): heading_schema,
            cv.Optional(CONF_MAGNITUDE): field_strength_schema,
            cv.Optional(CONF_OVERSAMPLING): cv.string,  # no-op; accepted for tronikos framework compatibility
            cv.Optional(CONF_SMOOTHING_FACTOR, default=0.0): cv.percentage,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x5E))
)

def auto_data_rate(config):
    interval_msec = config[CONF_UPDATE_INTERVAL].total_milliseconds
    interval_hz = 1000.0 / interval_msec
    for datarate in sorted(TLV493DDatarates.keys()):
        if float(datarate) >= interval_hz:
            return TLV493DDatarates[datarate]
    return TLV493DDatarates[75]

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_datarate(auto_data_rate(config)))
    cg.add(var.set_smoothing_factor(config[CONF_SMOOTHING_FACTOR]))
    if CONF_FIELD_STRENGTH_X in config:
        sens = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_X])
        cg.add(var.set_x_sensor(sens))
    if CONF_FIELD_STRENGTH_Y in config:
        sens = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_Y])
        cg.add(var.set_y_sensor(sens))
    if CONF_FIELD_STRENGTH_Z in config:
        sens = await sensor.new_sensor(config[CONF_FIELD_STRENGTH_Z])
        cg.add(var.set_z_sensor(sens))
    if CONF_HEADING in config:
        sens = await sensor.new_sensor(config[CONF_HEADING])
        cg.add(var.set_heading_sensor(sens))
    if CONF_MAGNITUDE in config:
        sens = await sensor.new_sensor(config[CONF_MAGNITUDE])
        cg.add(var.set_magnitude_sensor(sens))
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
