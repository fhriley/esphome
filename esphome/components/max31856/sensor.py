from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor, spi
import esphome.config_validation as cv
from esphome.const import (
    CONF_MAINS_FILTER,
    CONF_THERMOCOUPLE_TYPE,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome.core import TimePeriodMilliseconds
from esphome.types import ConfigType

from . import MAX31856Sensor, max31856_ns

CONF_DATA_READY_PIN = "data_ready_pin"
CONF_SAMPLES_PER_VALUE = "samples_per_value"

MAX31865ConfigFilter = max31856_ns.enum("MAX31856ConfigFilter")
FILTER = {
    50: MAX31865ConfigFilter.FILTER_50HZ,
    60: MAX31865ConfigFilter.FILTER_60HZ,
}
MAX31856ThermocoupleType = max31856_ns.enum("MAX31856ThermocoupleType")
THERMOCOUPLE_TYPE = MAX31856ThermocoupleType.options(
    "B", "E", "J", "K", "N", "R", "S", "T", prefix="MAX31856_TCTYPE_"
)

MAX31856SamplesPerValue = max31856_ns.enum("MAX31856SamplesPerValue")
SAMPLES_PER_VALUE = {
    1: MAX31856SamplesPerValue.AVE_SAMPLES_1,
    2: MAX31856SamplesPerValue.AVE_SAMPLES_2,
    4: MAX31856SamplesPerValue.AVE_SAMPLES_4,
    8: MAX31856SamplesPerValue.AVE_SAMPLES_8,
    16: MAX31856SamplesPerValue.AVE_SAMPLES_16,
}

EXCLUSIVE_ERR_MSG = "Use only one of data_ready_pin or update_interval"
DEFAULT_UPDATE_INTERVAL = TimePeriodMilliseconds(seconds=60)


def _default_update_interval(config: ConfigType) -> ConfigType:
    """Poll every 60s unless the data ready pin selects autoconversion mode."""
    if CONF_DATA_READY_PIN not in config and CONF_UPDATE_INTERVAL not in config:
        config[CONF_UPDATE_INTERVAL] = DEFAULT_UPDATE_INTERVAL
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        MAX31856Sensor,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_MAINS_FILTER, default="60Hz"): cv.All(
                cv.frequency, cv.enum(FILTER, int=True)
            ),
            cv.Optional(CONF_THERMOCOUPLE_TYPE, default="K"): cv.enum(
                THERMOCOUPLE_TYPE, upper=True, space=""
            ),
            cv.Optional(CONF_SAMPLES_PER_VALUE, default=1): cv.enum(
                SAMPLES_PER_VALUE, int=True
            ),
            cv.Exclusive(
                CONF_DATA_READY_PIN, "mode", EXCLUSIVE_ERR_MSG
            ): pins.gpio_input_pullup_pin_schema,
            cv.Exclusive(
                CONF_UPDATE_INTERVAL, "mode", EXCLUSIVE_ERR_MSG
            ): cv.update_interval,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema()),
    _default_update_interval,
)


async def to_code(config: ConfigType) -> None:
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_filter(config[CONF_MAINS_FILTER]))
    cg.add(var.set_thermocouple_type(config[CONF_THERMOCOUPLE_TYPE]))
    cg.add(var.set_samples_per_value(config[CONF_SAMPLES_PER_VALUE]))
    await cg.add_pin(var.set_data_ready_pin, config.get(CONF_DATA_READY_PIN))
    if (update_interval := config.get(CONF_UPDATE_INTERVAL)) is not None:
        cg.add(var.set_update_interval(update_interval))
