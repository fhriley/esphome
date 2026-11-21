import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_PROBLEM, ENTITY_CATEGORY_DIAGNOSTIC
from esphome.types import ConfigType

from . import CONF_MAX31865_ID, MAX31865Sensor

CONF_HAS_FAULT = "has_fault"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MAX31865_ID): cv.use_id(MAX31865Sensor),
        cv.Required(CONF_HAS_FAULT): binary_sensor.binary_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            device_class=DEVICE_CLASS_PROBLEM,
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_MAX31865_ID])
    sensors = binary_sensor.sub_binary_sensors(config)
    await sensors(CONF_HAS_FAULT, hub.set_has_fault_binary_sensor)
