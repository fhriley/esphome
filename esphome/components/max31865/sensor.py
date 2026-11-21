from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor, spi
import esphome.config_validation as cv
from esphome.const import (
    CONF_MAINS_FILTER,
    CONF_REFERENCE_RESISTANCE,
    CONF_RTD_NOMINAL_RESISTANCE,
    CONF_RTD_WIRES,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    SCHEDULER_DONT_RUN,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)
from esphome.core import TimePeriodMilliseconds
from esphome.types import ConfigType

from . import MAX31865Sensor, max31865_ns

CODEOWNERS = ["@DAVe3283"]
DEPENDENCIES = ["spi"]

CONF_DATA_READY_PIN = "data_ready_pin"

MAX31865ConfigFilter = max31865_ns.enum("MAX31865ConfigFilter")
FILTER = MAX31865ConfigFilter.options("50HZ", "60HZ", prefix="FILTER_")

EXCLUSIVE_ERR_MSG = "Use only one of data_ready_pin or update_interval"
DEFAULT_UPDATE_INTERVAL = TimePeriodMilliseconds(seconds=60)

# Datasheet maximum for a one-shot conversion, by mains filter
ONE_SHOT_MS = {"50HZ": 66, "60HZ": 55}
# V_BIAS settling before the fault detection cycle (BIAS_SETTLE_MS in max31865.cpp)
BIAS_SETTLE_MS = 11
# Fault detection cycle and input filter settling (FAULT_DETECTION_MS in max31865.cpp)
FAULT_DETECTION_MS = 7


def _conversion_time_ms(config: ConfigType) -> int:
    """Worst-case time for one one-shot conversion."""
    return ONE_SHOT_MS[str(config[CONF_MAINS_FILTER])]


def _measurement_time_ms(config: ConfigType) -> int:
    """Worst-case time from the start of a poll to its result."""
    return BIAS_SETTLE_MS + FAULT_DETECTION_MS + _conversion_time_ms(config)


def _validate_update_interval(config: ConfigType) -> ConfigType:
    """Poll every 60s unless the data ready pin selects autoconversion mode.

    A poll starts a new measurement, so the interval must be longer than one measurement.
    """
    if CONF_DATA_READY_PIN in config:
        return config
    if CONF_UPDATE_INTERVAL not in config:
        config[CONF_UPDATE_INTERVAL] = DEFAULT_UPDATE_INTERVAL
    interval = config[CONF_UPDATE_INTERVAL].total_milliseconds
    measurement_time = _measurement_time_ms(config)
    if interval != SCHEDULER_DONT_RUN and interval <= measurement_time:
        raise cv.Invalid(
            f"{CONF_UPDATE_INTERVAL} must be longer than the {measurement_time}ms a measurement takes "
            f"with {CONF_MAINS_FILTER} {config[CONF_MAINS_FILTER]}",
            path=[CONF_UPDATE_INTERVAL],
        )
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        MAX31865Sensor,
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Required(CONF_REFERENCE_RESISTANCE): cv.All(
                cv.resistance, cv.Range(min=100, max=10000)
            ),
            cv.Required(CONF_RTD_NOMINAL_RESISTANCE): cv.All(
                cv.resistance, cv.Range(min=100, max=1000)
            ),
            cv.Optional(CONF_MAINS_FILTER, default="60HZ"): cv.enum(
                FILTER, upper=True, space=""
            ),
            cv.Optional(CONF_RTD_WIRES, default=4): cv.int_range(min=2, max=4),
            cv.Exclusive(
                CONF_DATA_READY_PIN, "mode", EXCLUSIVE_ERR_MSG
            ): pins.gpio_input_pin_schema,
            cv.Exclusive(
                CONF_UPDATE_INTERVAL, "mode", EXCLUSIVE_ERR_MSG
            ): cv.update_interval,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema()),
    _validate_update_interval,
)


async def to_code(config: ConfigType) -> None:
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_reference_resistance(config[CONF_REFERENCE_RESISTANCE]))
    cg.add(var.set_nominal_resistance(config[CONF_RTD_NOMINAL_RESISTANCE]))
    cg.add(var.set_filter(config[CONF_MAINS_FILTER]))
    cg.add(var.set_num_rtd_wires(config[CONF_RTD_WIRES]))
    await cg.add_pin(var.set_data_ready_pin, config.get(CONF_DATA_READY_PIN))
    if (update_interval := config.get(CONF_UPDATE_INTERVAL)) is not None:
        cg.add(var.set_update_interval(update_interval))
        cg.add(var.set_conversion_time(_conversion_time_ms(config)))
